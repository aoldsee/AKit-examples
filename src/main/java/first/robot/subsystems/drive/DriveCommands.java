package first.robot.subsystems.drive;

import first.robot.field.AlignTargets;
import first.robot.field.FieldGeometry;
import first.robot.util.TunableNumber;
import first.robot.util.VectorRateLimiter;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Command;
import org.wpilib.math.controller.ProfiledPIDController;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.trajectory.TrapezoidProfile;
import org.wpilib.math.util.MathUtil;
import org.wpilib.system.Timer;

/**
 * Commands that drive the robot somewhere: joystick driving, fixed moves, and DriveToPose. The
 * routines that measure the drivetrain are in {@link DriveCharacterization} and {@link
 * DriveSystemsCheck}.
 */
public final class DriveCommands {
  private static final double DEADBAND = 0.1;

  // Driver acceleration limits in m/s^2, editable under /Tuning/Drive/ in tuning mode. Shared by
  // every joystick drive command.
  private static final TunableNumber maxAccel =
      new TunableNumber("Drive/MaxAccelMetersPerSec2", DriveConstants.DRIVER_MAX_ACCEL);
  private static final TunableNumber maxDecel =
      new TunableNumber("Drive/MaxDecelMetersPerSec2", DriveConstants.DRIVER_MAX_DECEL);

  // DriveToPose feedback gains: m/s per meter of distance error, rad/s per radian of heading
  // error. The motion profiles do most of the work, so these only clean up what's left.
  private static final double ALIGN_DISTANCE_KP = 3.0;
  private static final double ALIGN_ANGLE_KP = 4.0;

  // Heading controller for joystickDriveAtAngle. Rad, rad/s, rad/s^2.
  private static final double ANGLE_KP = 5.0;
  private static final double ANGLE_KD = 0.4;
  private static final double ANGLE_MAX_VELOCITY = 8.0;
  private static final double ANGLE_MAX_ACCELERATION = 20.0;

  private static final DoubleSupplier NO_ACCEL_CAP = () -> Double.POSITIVE_INFINITY;

  private DriveCommands() {}

  /**
   * Field-relative drive. Inputs are joystick axes in [-1, 1], already sign-corrected so +x is away
   * from the driver and +y is to the driver's left.
   */
  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return joystickDrive(drive, xSupplier, ySupplier, omegaSupplier, NO_ACCEL_CAP);
  }

  /**
   * As above, with an extra acceleration cap in m/s^2 that can change while driving. It's checked
   * every loop, so another mechanism can tighten it on the fly (Controls does this while the arm is
   * raised). Return infinity for no extra cap. The shorter version above just calls this one with
   * no cap; two methods with the same name and different parameters are called overloads.
   */
  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier,
      DoubleSupplier accelCap) {
    var limiter = limiter(accelCap);
    return drive
        .run(
            coroutine -> {
              // Start from how the robot is actually moving, not wherever the limiter was left.
              limiter.reset(measuredDriverVelocity(drive));
              // Never ends on its own. yield() pauses until the next loop, so this doesn't freeze
              // the robot; the command stops when canceled (a button released, or another
              // command taking the drive).
              while (true) {
                var linear = limitedLinearVelocity(limiter, xSupplier, ySupplier);
                // Omega is turning speed, in radians per second.
                double omega =
                    rotationFromJoystick(omegaSupplier.getAsDouble())
                        * DriveConstants.MAX_ANGULAR_SPEED;
                // The sticks ask for field directions; the drive needs the robot's own. This is
                // the line that makes driving field-relative.
                drive.runVelocity(
                    new ChassisVelocities(linear.getX(), linear.getY(), omega)
                        .toRobotRelative(driverRelativeHeading(drive)));
                coroutine.yield();
              }
            })
        .named("Drive.Joystick");
  }

  /** Field-relative translation from the sticks, heading held by a profiled PID controller. */
  public static Command joystickDriveAtAngle(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Rotation2d> headingSupplier) {
    return joystickDriveAtAngle(drive, xSupplier, ySupplier, headingSupplier, NO_ACCEL_CAP);
  }

  /** As above, with an extra acceleration cap like {@link #joystickDrive}'s. */
  public static Command joystickDriveAtAngle(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Rotation2d> headingSupplier,
      DoubleSupplier accelCap) {
    // A ProfiledPIDController is a PID controller with a motion profile: instead of chasing the
    // target heading directly, it plans a smooth turn (speed up, cruise, slow down) within the max
    // velocity and acceleration below, and the PID follows that plan. A "trapezoid" profile is
    // named for the shape of its speed graph.
    var angleController =
        new ProfiledPIDController(
            ANGLE_KP,
            0.0,
            ANGLE_KD,
            new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    // Angles wrap around: 179 degrees and -179 degrees are only 2 degrees apart. Without this, the
    // controller would turn the long way around.
    angleController.enableContinuousInput(-Math.PI, Math.PI);
    var limiter = limiter(accelCap);

    return drive
        .run(
            coroutine -> {
              // Start the profile from the current heading and turning speed, so taking over
              // mid-turn doesn't lurch.
              angleController.reset(
                  drive.getRotation().getRadians(), drive.getChassisVelocities().omega);
              limiter.reset(measuredDriverVelocity(drive));
              // Never ends on its own. yield() pauses until the next loop, so this doesn't freeze
              // the robot; the command stops when canceled (a button released, or another
              // command taking the drive).
              while (true) {
                var linear = limitedLinearVelocity(limiter, xSupplier, ySupplier);
                // Two parts. The profile's planned turning speed does most of the work: it's how
                // fast the plan says to be turning right now. PID only corrects the gap between
                // the plan and where the robot actually points. With PID alone, the robot can only
                // turn once it's already behind the plan, so it always trails it.
                double omega =
                    angleController.calculate(
                            drive.getRotation().getRadians(), headingSupplier.get().getRadians())
                        + angleController.getSetpoint().velocity;
                drive.runVelocity(
                    new ChassisVelocities(linear.getX(), linear.getY(), omega)
                        .toRobotRelative(driverRelativeHeading(drive)));
                coroutine.yield();
              }
            })
        .named("Drive.JoystickAtAngle");
  }

  /** Drives robot-relative at a fixed velocity for a fixed time, then stops. */
  public static Command driveFor(Drive drive, ChassisVelocities velocities, double seconds) {
    return drive
        .run(
            coroutine -> {
              var timer = Timer.createStarted();
              while (!timer.hasElapsed(seconds)) {
                drive.runVelocity(velocities);
                coroutine.yield();
              }
              drive.stop();
            })
        .named("Drive.DriveFor");
  }

  /**
   * Drives to a field pose and finishes once there and stopped.
   *
   * <p>Two profiled controllers do the work. One controls the straight-line distance to the target,
   * which keeps the path straight instead of correcting x and y separately and curving. The other
   * controls heading. Profiles mean the robot speeds up and slows down smoothly instead of lurching
   * at full output, and arrives with little overshoot.
   *
   * <p>The target is read once, when the command starts. Poses are field coordinates (blue origin),
   * so unlike joystick driving there's no alliance flip here.
   */
  public static Command driveToPose(Drive drive, Supplier<Pose2d> targetSupplier) {
    var distanceController =
        new ProfiledPIDController(
            ALIGN_DISTANCE_KP,
            0.0,
            0.0,
            new TrapezoidProfile.Constraints(
                DriveConstants.ALIGN_MAX_VELOCITY, DriveConstants.ALIGN_MAX_ACCEL));
    var angleController =
        new ProfiledPIDController(
            ALIGN_ANGLE_KP,
            0.0,
            0.0,
            new TrapezoidProfile.Constraints(
                DriveConstants.ALIGN_MAX_ANGULAR_VELOCITY, DriveConstants.ALIGN_MAX_ANGULAR_ACCEL));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    return drive
        .run(
            coroutine -> {
              var target = targetSupplier.get();
              Logger.recordOutput("DriveToPose/Target", target);

              // Start both profiles from how the robot is moving now, so taking over from the
              // driver mid-motion is smooth. Only the speed toward the target carries over; any
              // sideways speed stops at once.
              var pose = drive.getPose();
              var fieldVelocity = drive.getChassisVelocities().toFieldRelative(drive.getRotation());
              var toTarget = target.getTranslation().minus(pose.getTranslation());
              double speedTowardTarget =
                  toTarget.getNorm() > 1e-6
                      ? (fieldVelocity.vx * toTarget.getX() + fieldVelocity.vy * toTarget.getY())
                          / toTarget.getNorm()
                      : 0.0;
              // Distance shrinks as the robot approaches, so its rate is the negative of speed.
              distanceController.reset(toTarget.getNorm(), -speedTowardTarget);
              angleController.reset(pose.getRotation().getRadians(), fieldVelocity.omega);

              while (true) {
                pose = drive.getPose();
                toTarget = target.getTranslation().minus(pose.getTranslation());
                double distance = toTarget.getNorm();
                double angleError = target.getRotation().minus(pose.getRotation()).getRadians();
                if (AlignTargets.isAligned(pose, drive.getChassisVelocities(), target)) {
                  break;
                }

                // The controller drives distance toward 0. Its output, plus the profile's
                // planned speed, is negative while closing in, so flip it to get a speed toward
                // the target. The profile's speed does most of the work; PID fixes the rest.
                double speed =
                    -(distanceController.calculate(distance, 0.0)
                        + distanceController.getSetpoint().velocity);
                var velocity =
                    distance > 1e-6 ? toTarget.times(speed / distance) : Translation2d.ZERO;
                double omega =
                    angleController.calculate(
                            pose.getRotation().getRadians(), target.getRotation().getRadians())
                        + angleController.getSetpoint().velocity;

                Logger.recordOutput("DriveToPose/DistanceError", distance);
                Logger.recordOutput("DriveToPose/AngleErrorDeg", Math.toDegrees(angleError));
                drive.runVelocity(
                    new ChassisVelocities(velocity.getX(), velocity.getY(), omega)
                        .toRobotRelative(pose.getRotation()));
                coroutine.yield();
              }
              drive.stop();
            })
        .named("Drive.DriveToPose");
  }

  /** Drives to the spot in front of the nearest AprilTag, picked when the command starts. */
  public static Command alignToNearestTag(Drive drive) {
    return driveToPose(drive, () -> AlignTargets.nearestTagOrCurrent(drive.getPose()));
  }

  /**
   * The driver's acceleration limiter: the tunable limits, tightened to {@code accelCap} whenever
   * that's lower. Both are read every loop, so a cap that changes mid-drive takes effect at once.
   */
  private static VectorRateLimiter limiter(DoubleSupplier accelCap) {
    return new VectorRateLimiter(
        () -> Math.min(maxAccel.get(), accelCap.getAsDouble()),
        () -> Math.min(maxDecel.get(), accelCap.getAsDouble()));
  }

  /** Stick translation in m/s in the driver's frame, after the acceleration limits. */
  private static Translation2d limitedLinearVelocity(
      VectorRateLimiter limiter, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
    var requested =
        linearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble())
            .times(DriveConstants.MAX_LINEAR_SPEED);
    return limiter.calculate(requested);
  }

  /** The robot's measured velocity, in the same driver frame the sticks command. */
  private static Translation2d measuredDriverVelocity(Drive drive) {
    var velocities = drive.getChassisVelocities().toFieldRelative(driverRelativeHeading(drive));
    return new Translation2d(velocities.vx, velocities.vy);
  }

  /**
   * Heading to treat as "forward" for field-relative driving. On red, the driver faces the other
   * way down the field, so the frame flips.
   */
  private static Rotation2d driverRelativeHeading(Drive drive) {
    return FieldGeometry.isRed()
        ? drive.getRotation().plus(Rotation2d.k180deg)
        : drive.getRotation();
  }

  /**
   * Turns stick axes into a translation in [-1, 1]. The deadband applies to the stick's magnitude
   * rather than per axis, so diagonals don't snap to the axes, and the magnitude is squared for
   * finer control at low speed.
   *
   * <p>Package-private so the unit tests can check it directly.
   */
  static Translation2d linearVelocityFromJoysticks(double x, double y) {
    // Many gamepads report close to (1, 1) in the corners, a magnitude of 1.41. Without the clamp
    // that asks for 141% speed, and desaturating the wheels then steals turning authority.
    double magnitude = MathUtil.applyDeadband(Math.min(1.0, Math.hypot(x, y)), DEADBAND);
    if (magnitude == 0.0) {
      return Translation2d.ZERO;
    }
    double direction = Math.atan2(y, x);
    magnitude *= magnitude;
    return new Translation2d(magnitude * Math.cos(direction), magnitude * Math.sin(direction));
  }

  /**
   * Turns a stick axis into a rotation command in [-1, 1]: deadband, then square, keeping the sign.
   * Squaring gives finer control near center without lowering top speed.
   */
  static double rotationFromJoystick(double value) {
    double omega = MathUtil.applyDeadband(value, DEADBAND);
    return Math.copySign(omega * omega, omega);
  }
}
