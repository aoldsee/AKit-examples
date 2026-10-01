package first.robot.commands;

import first.robot.subsystems.drive.Drive;
import first.robot.subsystems.drive.DriveConstants;
import first.robot.util.FieldGeometry;
import first.robot.util.TunableNumber;
import first.robot.util.VectorRateLimiter;
import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.DoubleUnaryOperator;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Command;
import org.wpilib.math.controller.ProfiledPIDController;
import org.wpilib.math.filter.SlewRateLimiter;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.trajectory.TrapezoidProfile;
import org.wpilib.math.util.MathUtil;
import org.wpilib.math.util.Units;
import org.wpilib.sysid.SysIdRoutineLog;
import org.wpilib.system.Timer;

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

  // Seconds to hold zero output so modules can finish turning before data collection.
  private static final double FF_START_DELAY = 2.0;

  // Volts per second. Slow enough that acceleration is negligible and kA can be ignored.
  private static final double FF_RAMP_RATE = 0.1;

  // Wheel rotations/s. Samples below this are static friction, not the linear region, and would
  // drag the fit toward a lower kS and higher kV.
  private static final double FF_MIN_VELOCITY = 0.01;

  // Rad/s and rad/s^2. Slow spin keeps wheels from slipping, which would inflate the radius.
  private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25;

  private static final double WHEEL_RADIUS_RAMP_RATE = 0.05;

  // SysId test parameters, matching WPILib's SysIdRoutine defaults. Volts/s, volts, seconds.
  private static final double SYSID_RAMP_RATE = 1.0;
  private static final double SYSID_STEP_VOLTAGE = 7.0;
  private static final double SYSID_TIMEOUT = 10.0;
  // Seconds at zero output before each test so the modules finish turning straight.
  private static final double SYSID_SETTLE_TIME = 1.0;

  private static final String SYSID_STATE_KEY = "Drive/SysIdState";

  private static final DoubleSupplier NO_ACCEL_CAP = () -> Double.POSITIVE_INFINITY;

  public enum SysIdDirection {
    FORWARD,
    REVERSE
  }

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
   * every loop, so another mechanism can tighten it on the fly (RobotContainer does this while the
   * arm is raised). Return infinity for no extra cap.
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
              while (true) {
                var linear = limitedLinearVelocity(limiter, xSupplier, ySupplier);
                double omega =
                    rotationFromJoystick(omegaSupplier.getAsDouble())
                        * DriveConstants.MAX_ANGULAR_SPEED;
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
    var angleController =
        new ProfiledPIDController(
            ANGLE_KP,
            0.0,
            ANGLE_KD,
            new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);
    var limiter = limiter(accelCap);

    return drive
        .run(
            coroutine -> {
              // Starting the profile from the current heading avoids a lurch on the first cycle.
              angleController.reset(drive.getRotation().getRadians());
              limiter.reset(measuredDriverVelocity(drive));
              while (true) {
                var linear = limitedLinearVelocity(limiter, xSupplier, ySupplier);
                double omega =
                    angleController.calculate(
                        drive.getRotation().getRadians(), headingSupplier.get().getRadians());
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
   * at full output, and arrives without overshooting.
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
              // driver mid-motion doesn't jerk.
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

  /**
   * Ramps drive voltage and fits kS and kV for the drive motors. Runs until canceled, then prints
   * the fit. Only meaningful with Voltage closed-loop output.
   */
  public static Command feedforwardCharacterization(Drive drive) {
    return feedforwardCharacterization(drive, DriveCommands::printFeedforwardFit);
  }

  /** As above, but hands the fit to {@code onResult} instead of printing it. */
  public static Command feedforwardCharacterization(
      Drive drive, Consumer<FeedforwardFit> onResult) {
    List<Double> velocitySamples = new ArrayList<>();
    List<Double> voltageSamples = new ArrayList<>();

    return drive
        .run(
            coroutine -> {
              velocitySamples.clear();
              voltageSamples.clear();

              var timer = Timer.createStarted();
              while (!timer.hasElapsed(FF_START_DELAY)) {
                drive.runCharacterization(0.0);
                coroutine.yield();
              }

              timer.restart();
              while (true) {
                double voltage = timer.get() * FF_RAMP_RATE;
                drive.runCharacterization(voltage);
                double velocity = drive.getFFCharacterizationVelocity();
                if (Math.abs(velocity) > FF_MIN_VELOCITY) {
                  velocitySamples.add(velocity);
                  voltageSamples.add(voltage);
                }
                coroutine.yield();
              }
            })
        .whenCanceled(
            () -> {
              if (velocitySamples.size() < 2) {
                System.out.println("Drive FF characterization canceled before collecting data.");
                return;
              }
              onResult.accept(fitFeedforward(velocitySamples, voltageSamples));
            })
        .named("Drive.FeedforwardCharacterization");
  }

  /**
   * Spins in place and compares gyro rotation with wheel rotation to measure the effective wheel
   * radius. Runs until canceled, then prints the result. Needs at least one full rotation.
   */
  public static Command wheelRadiusCharacterization(Drive drive) {
    return wheelRadiusCharacterization(drive, DriveCommands::printWheelRadius);
  }

  /** As above, but hands the result to {@code onResult} instead of printing it. */
  public static Command wheelRadiusCharacterization(
      Drive drive, Consumer<WheelRadiusResult> onResult) {
    var limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
    var state = new WheelRadiusState();

    return drive
        .run(
            coroutine -> {
              limiter.reset(0.0);
              state.started = false;

              var timer = Timer.createStarted();
              while (true) {
                double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                drive.runVelocity(new ChassisVelocities(0.0, 0.0, speed));

                // The first second lets the modules finish turning to the spin orientation.
                if (!state.started && timer.hasElapsed(1.0)) {
                  state.started = true;
                  state.startPositions = drive.getWheelRadiusCharacterizationPositions();
                  state.lastAngle = drive.getRotation();
                  state.gyroDelta = 0.0;
                } else if (state.started) {
                  var rotation = drive.getRotation();
                  state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                  state.lastAngle = rotation;
                }
                coroutine.yield();
              }
            })
        .whenCanceled(
            () -> {
              if (!state.started) {
                System.out.println(
                    "Wheel radius characterization canceled before collecting data.");
                return;
              }
              onResult.accept(measureWheelRadius(drive, state));
            })
        .named("Drive.WheelRadiusCharacterization");
  }

  /**
   * SysId quasistatic test: drive voltage ramps slowly, so velocity tracks voltage and kS/kV can be
   * fit. Load the AdvantageKit log into the SysId tool and pick {@code Drive/SysIdState} as the
   * test state.
   */
  public static Command sysIdQuasistatic(Drive drive, SysIdDirection direction) {
    boolean forward = direction == SysIdDirection.FORWARD;
    return sysIdTest(
        drive,
        forward
            ? SysIdRoutineLog.State.QUASISTATIC_FORWARD
            : SysIdRoutineLog.State.QUASISTATIC_REVERSE,
        t -> (forward ? 1 : -1) * SYSID_RAMP_RATE * t);
  }

  /** SysId dynamic test: a voltage step, so the acceleration response gives kA. */
  public static Command sysIdDynamic(Drive drive, SysIdDirection direction) {
    boolean forward = direction == SysIdDirection.FORWARD;
    return sysIdTest(
        drive,
        forward ? SysIdRoutineLog.State.DYNAMIC_FORWARD : SysIdRoutineLog.State.DYNAMIC_REVERSE,
        t -> (forward ? 1 : -1) * SYSID_STEP_VOLTAGE);
  }

  /**
   * Shared body of the SysId tests. WPILib's SysIdRoutine is built on Commands v2, so this does the
   * same thing as a v3 coroutine. AdvantageKit already logs the voltage, position, and velocity
   * inputs SysId needs; only the test state is recorded here.
   */
  private static Command sysIdTest(
      Drive drive, SysIdRoutineLog.State state, DoubleUnaryOperator voltageAtTime) {
    return drive
        .run(
            coroutine -> {
              var timer = Timer.createStarted();
              while (!timer.hasElapsed(SYSID_SETTLE_TIME)) {
                drive.runCharacterization(0.0);
                Logger.recordOutput(SYSID_STATE_KEY, SysIdRoutineLog.State.NONE.toString());
                coroutine.yield();
              }

              timer.restart();
              while (!timer.hasElapsed(SYSID_TIMEOUT)) {
                drive.runCharacterization(voltageAtTime.applyAsDouble(timer.get()));
                Logger.recordOutput(SYSID_STATE_KEY, state.toString());
                coroutine.yield();
              }

              drive.runCharacterization(0.0);
              Logger.recordOutput(SYSID_STATE_KEY, SysIdRoutineLog.State.NONE.toString());
            })
        // Without this the log would show the test still running after a cancel.
        .whenCanceled(
            () -> Logger.recordOutput(SYSID_STATE_KEY, SysIdRoutineLog.State.NONE.toString()))
        .named("Drive.SysId[" + state + "]");
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

  /** kV is volts per wheel rotation/s, matching the drive gains in TunerConstants. */
  public record FeedforwardFit(double kS, double kV) {}

  /** Wheel and gyro deltas in radians, radius in meters. */
  public record WheelRadiusResult(double wheelDeltaRad, double gyroDeltaRad, double radiusMeters) {}

  /**
   * Turns a stick axis into a rotation command in [-1, 1]: deadband, then square, keeping the sign.
   * Squaring gives finer control near center without lowering top speed.
   */
  static double rotationFromJoystick(double value) {
    double omega = MathUtil.applyDeadband(value, DEADBAND);
    return Math.copySign(omega * omega, omega);
  }

  /**
   * Least-squares fit of voltage = kS + kV * velocity: the same line of best fit a spreadsheet
   * trendline draws through the points. kS is where the line crosses zero velocity (the voltage
   * needed just to overcome friction), and kV is its slope.
   */
  private static FeedforwardFit fitFeedforward(List<Double> velocities, List<Double> voltages) {
    int n = velocities.size();
    double sumX = 0.0;
    double sumY = 0.0;
    double sumXY = 0.0;
    double sumX2 = 0.0;
    for (int i = 0; i < n; i++) {
      double x = velocities.get(i);
      double y = voltages.get(i);
      sumX += x;
      sumY += y;
      sumXY += x * y;
      sumX2 += x * x;
    }
    double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
    double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);
    return new FeedforwardFit(kS, kV);
  }

  private static WheelRadiusResult measureWheelRadius(Drive drive, WheelRadiusState state) {
    double[] positions = drive.getWheelRadiusCharacterizationPositions();
    double wheelDelta = 0.0;
    for (int i = 0; i < positions.length; i++) {
      wheelDelta += Math.abs(positions[i] - state.startPositions[i]) / positions.length;
    }
    // Each wheel travels the arc the robot turns through at the drive base radius.
    double radius = (state.gyroDelta * DriveConstants.DRIVE_BASE_RADIUS) / wheelDelta;
    return new WheelRadiusResult(wheelDelta, state.gyroDelta, radius);
  }

  private static void printFeedforwardFit(FeedforwardFit fit) {
    var formatter = new DecimalFormat("#0.00000");
    System.out.println("********** Drive FF Characterization Results **********");
    System.out.println("\tkS: " + formatter.format(fit.kS()));
    System.out.println("\tkV: " + formatter.format(fit.kV()) + " (V per wheel rotation/s)");
  }

  private static void printWheelRadius(WheelRadiusResult result) {
    var formatter = new DecimalFormat("#0.000");
    System.out.println("********** Wheel Radius Characterization Results **********");
    System.out.println("\tWheel Delta: " + formatter.format(result.wheelDeltaRad()) + " radians");
    System.out.println("\tGyro Delta: " + formatter.format(result.gyroDeltaRad()) + " radians");
    System.out.println(
        "\tWheel Radius: "
            + formatter.format(result.radiusMeters())
            + " meters, "
            + formatter.format(Units.metersToInches(result.radiusMeters()))
            + " inches");
  }

  private static final class WheelRadiusState {
    boolean started = false;
    double[] startPositions = new double[4];
    Rotation2d lastAngle = Rotation2d.ZERO;
    double gyroDelta = 0.0;
  }
}
