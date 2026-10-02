package first.robot.subsystems.drive;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import first.robot.Constants;
import first.robot.hardware.MK5n;
import first.robot.hardware.Motors;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N3;
import org.wpilib.math.util.Units;

/**
 * Everything about the drivetrain that's a fixed number: wiring, gearing, gains, current limits,
 * and how the robot should drive.
 *
 * <p>This describes a placeholder robot: SDS MK5n R2 modules with Kraken X60 drive and Kraken X44
 * steer motors, CANcoders, and a Pigeon 2. Gear ratios, wheel size, and feedforward gains are
 * worked out from the specs in {@link MK5n} and {@link Motors}, so changing a module or motor there
 * updates everything here. For a real robot, the per-module numbers in {@link #MODULES} (CAN IDs,
 * encoder offsets, inverts) come from the robot itself; Phoenix Tuner X's swerve setup screens are
 * a convenient way to find them.
 */
public final class DriveConstants {
  private DriveConstants() {}

  // ---------------------------------------------------------------------------------------------
  // Wiring

  /**
   * All swerve devices share one bus. CAN_S0 is a SystemCore built-in CAN FD port, which is fast
   * enough for 250 Hz odometry.
   */
  public static final CANBus CAN_BUS = new CANBus(CANPort.CAN_S0);

  public static final int PIGEON_ID = 13;

  /**
   * What differs between the four modules. Everything else is shared and listed below.
   *
   * @param encoderOffsetRotations added to the CANcoder reading so the module reads 0 with the
   *     wheel pointing straight ahead
   * @param location where the module sits, meters from the robot's center (+x forward, +y left)
   */
  public record ModuleConfig(
      int driveMotorId,
      int steerMotorId,
      int encoderId,
      double encoderOffsetRotations,
      Translation2d location,
      InvertedValue driveMotorDirection,
      InvertedValue steerMotorDirection,
      SensorDirectionValue encoderDirection) {}

  // Modules sit this far from the center in both directions. The right side's drive motors count
  // clockwise as positive: they're mounted mirrored, so the left side's direction would push the
  // robot backward.
  private static final double MODULE_OFFSET_METERS = Units.inchesToMeters(11.5);

  /**
   * The four modules in FL, FR, BL, BR order. Every module-indexed array uses this order. CAN IDs
   * go drive, steer, encoder for each module in turn.
   */
  public static final ModuleConfig[] MODULES = {
    new ModuleConfig(
        1,
        2,
        3,
        0.0,
        new Translation2d(MODULE_OFFSET_METERS, MODULE_OFFSET_METERS),
        InvertedValue.CounterClockwise_Positive,
        InvertedValue.Clockwise_Positive,
        SensorDirectionValue.CounterClockwise_Positive),
    new ModuleConfig(
        4,
        5,
        6,
        0.0,
        new Translation2d(MODULE_OFFSET_METERS, -MODULE_OFFSET_METERS),
        InvertedValue.Clockwise_Positive,
        InvertedValue.Clockwise_Positive,
        SensorDirectionValue.CounterClockwise_Positive),
    new ModuleConfig(
        7,
        8,
        9,
        0.0,
        new Translation2d(-MODULE_OFFSET_METERS, MODULE_OFFSET_METERS),
        InvertedValue.CounterClockwise_Positive,
        InvertedValue.Clockwise_Positive,
        SensorDirectionValue.CounterClockwise_Positive),
    new ModuleConfig(
        10,
        11,
        12,
        0.0,
        new Translation2d(-MODULE_OFFSET_METERS, -MODULE_OFFSET_METERS),
        InvertedValue.Clockwise_Positive,
        InvertedValue.Clockwise_Positive,
        SensorDirectionValue.CounterClockwise_Positive)
  };

  // ---------------------------------------------------------------------------------------------
  // Hardware

  public static final Motors.Spec DRIVE_MOTOR = Motors.KRAKEN_X60_FOC;
  public static final Motors.Spec STEER_MOTOR = Motors.KRAKEN_X44_FOC;

  /** Drive motor rotations per wheel rotation. */
  public static final double DRIVE_GEAR_RATIO = MK5n.DRIVE_RATIO_R2;

  /** Steer motor rotations per module rotation. */
  public static final double STEER_GEAR_RATIO = MK5n.STEER_RATIO;

  /** Drive motor rotations per module rotation, from the bevel gear (see MK5n.COUPLING_RATIO). */
  public static final double COUPLING_GEAR_RATIO = MK5n.COUPLING_RATIO;

  public static final double WHEEL_RADIUS_METERS = MK5n.WHEEL_RADIUS_METERS;

  /**
   * How well the tread grips the carpet (coefficient of friction). A wheel can push up to this
   * times the weight on it before it slips. The simulator uses it; on a real robot it's why
   * SLIP_CURRENT_AMPS exists.
   */
  public static final double WHEEL_COF = MK5n.WHEEL_MOLDED_COF_CARPET;

  /**
   * The steer motor closes its loop on the CANcoder. Fused uses the CANcoder for absolute position
   * and the motor's own sensor for smooth fine detail. It needs Phoenix Pro; without a license it
   * silently falls back to RemoteCANcoder (the CANcoder alone).
   */
  public static final FeedbackSensorSourceValue STEER_FEEDBACK =
      FeedbackSensorSourceValue.FusedCANcoder;

  // ---------------------------------------------------------------------------------------------
  // Characterization

  // How each motor's mechanism responds to voltage, in the form characterization measures:
  //   kS: volts just to overcome friction
  //   kV: volts per unit of speed, once moving
  //   kA: volts per unit of acceleration
  // These are the feedforward gains, and they're also what the simulator builds its model from.
  // After characterizing the real robot, put the measured values here, and both the robot's
  // control and the simulator match the real robot. Until then they're estimates worked out from
  // the motor specs and the robot's mass.
  //
  // Drive units are wheel rotations (V per rotation/s, V per rotation/s^2); steer units are module
  // rotations.

  /** Holding still, a motor's voltage is current * resistance, and its current is torque / Kt. */
  private static final double DRIVE_VOLTS_PER_WHEEL_NM =
      DRIVE_MOTOR.resistanceOhms() / (DRIVE_MOTOR.torquePerAmp() * DRIVE_GEAR_RATIO);

  private static final double STEER_VOLTS_PER_MODULE_NM =
      STEER_MOTOR.resistanceOhms() / (STEER_MOTOR.torquePerAmp() * STEER_GEAR_RATIO);

  /**
   * A typical value for a swerve module, not worked out from anything. Replace with the measured
   * one.
   */
  public static final double DRIVE_KS = 0.2;

  /** The motor turns rpsPerVolt per volt, and the wheel turns 1 / DRIVE_GEAR_RATIO as fast. */
  public static final double DRIVE_KV = DRIVE_GEAR_RATIO / DRIVE_MOTOR.rpsPerVolt();

  /**
   * Each drive motor has to accelerate a quarter of the robot, driving straight. A mass m moving
   * with the edge of a wheel of radius r resists like an inertia of m * r^2. Torque for 1
   * rotation/s^2 is that inertia * 2 pi, converted to volts. Spinning is different, because the
   * mass is spread out instead of sitting at the wheels; that's ROBOT_MOI_KG_M2.
   */
  public static final double DRIVE_KA =
      Constants.ROBOT_MASS_KG
          / 4.0
          * WHEEL_RADIUS_METERS
          * WHEEL_RADIUS_METERS
          * 2
          * Math.PI
          * DRIVE_VOLTS_PER_WHEEL_NM;

  public static final double STEER_KS = 0.2;
  public static final double STEER_KV = STEER_GEAR_RATIO / STEER_MOTOR.rpsPerVolt();

  /** The module turning about its axis, estimated at 0.004 kg*m^2. */
  public static final double STEER_KA = 0.004 * 2 * Math.PI * STEER_VOLTS_PER_MODULE_NM;

  // ---------------------------------------------------------------------------------------------
  // Control

  /** How a motor's closed loop drives it. See the comment on the requests in ModuleIOTalonFX. */
  public enum ClosedLoopOutput {
    VOLTAGE,
    TORQUE_CURRENT_FOC
  }

  public static final ClosedLoopOutput DRIVE_CLOSED_LOOP_OUTPUT = ClosedLoopOutput.VOLTAGE;
  public static final ClosedLoopOutput STEER_CLOSED_LOOP_OUTPUT = ClosedLoopOutput.VOLTAGE;

  // Steer gains are per module rotation because the CANcoder is the feedback sensor. kP and kD
  // are tuned, not measured.
  public static final Slot0Configs STEER_GAINS =
      new Slot0Configs()
          .withKP(100)
          .withKI(0)
          .withKD(0.5)
          .withKS(STEER_KS)
          .withKV(STEER_KV)
          .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

  // Drive gains are per wheel rotation, because ModuleIOTalonFX sets SensorToMechanismRatio to the
  // drive reduction. kS matters more than it looks: without it, the tiny speeds asked for in the
  // last centimeter of a DriveToPose don't overcome friction, and the robot stalls just short of
  // the target. kA isn't set, because the Talon only uses it when a request includes an
  // acceleration, and these don't.
  //
  // kP starts from the common default of 0.1 V per rotation/s of error, which is per *motor*
  // rotation. One wheel rotation is DRIVE_GEAR_RATIO motor rotations, so the same error measured
  // in wheel rotations is a number 6 times smaller. Multiplying by the ratio keeps the motor
  // pushing just as hard for the same speed error. Any gain converted between motor and mechanism
  // units needs this
  public static final Slot0Configs DRIVE_GAINS =
      new Slot0Configs()
          .withKP(0.1 * DRIVE_GEAR_RATIO)
          .withKI(0)
          .withKD(0)
          .withKS(DRIVE_KS)
          .withKV(DRIVE_KV);

  // ---------------------------------------------------------------------------------------------
  // Current limits

  /**
   * Drive stator current limit, amps. This more directly limits torque of the motor.
   *
   * <p>Set just under the current where the wheels slip. Each wheel can push WHEEL_COF times its
   * quarter of the robot's weight, about 350 N here, and each amp pushes about 2.3 N (motor Kt *
   * gear ratio / wheel radius), so the tread lets go at about 150 A. Worn tread or a lighter robot
   * slips sooner, so lower this if the wheels spin on a hard launch.
   */
  public static final double SLIP_CURRENT_AMPS = 150.0;

  /**
   * Drive supply current limit, amps. Supply current is what the battery sees, and it's what causes
   * brownouts.
   *
   * <p>The cost of a lower limit is a slower launch. See
   * DriveSimTest.hardAccelerationDoesNotBrownOut, and try Battery.RESISTANCE_OHMS = 0.030 (worn) or
   * a half-charged new Battery(18.0, 0.5).
   */
  public static final double DRIVE_SUPPLY_CURRENT_LIMIT_AMPS = 40.0;

  /** Steer stator current limit, amps. Steering needs little torque. */
  public static final double STEER_STATOR_CURRENT_LIMIT_AMPS = 60.0;

  // ---------------------------------------------------------------------------------------------
  // Derived values and driving behavior

  /** Hz. CAN 2.0 can't carry 250 Hz of drive and steer signals for four modules. */
  public static final double ODOMETRY_FREQUENCY = CAN_BUS.isNetworkFD() ? 250.0 : 100.0;

  /**
   * How much the pose estimator trusts odometry, as standard deviations (meters, meters, radians).
   *
   * <p>Each vision frame moves the estimate a fixed fraction of the way toward the camera's answer:
   * odometry / (odometry + vision), using these standard deviations directly (not squared). That's
   * WPILib's simple rule, not a full Kalman filter, so odometry's uncertainty doesn't grow as the
   * robot drives. This number is a tuning knob, not something to measure. And since the fraction is
   * the same however often frames arrive, it has to be picked together with the camera's frame
   * rate:
   *
   * <ul>
   *   <li>Too large a fraction, and the pose jumps with each frame's noise. With WPILib's default
   *       of 0.1 m, a typical frame here (5 tags at 4 m, about 1 cm) moves the estimate over 90% of
   *       the way.
   *   <li>Too small, and a real error (a bump, a wheel slipping) takes a long time to fix.
   * </ul>
   *
   * <p>At 0.002 m a typical frame moves it about 20% of the way. At 90 frames a second that still
   * fixes a real error in well under a second, while averaging out most of the noise. A camera
   * running at 30 fps would need about 3 times this value to correct as quickly.
   */
  public static final Matrix<N3, N1> ODOMETRY_STD_DEVS = VecBuilder.fill(0.002, 0.002, 0.002);

  /** Robot-relative module positions, meters. */
  public static final Translation2d[] MODULE_TRANSLATIONS = moduleTranslations();

  /** Meters, center to the farthest module. */
  public static final double DRIVE_BASE_RADIUS = driveBaseRadius();

  /**
   * m/s. Theoretical top speed: motor free speed, through the reduction, times the wheel's
   * circumference.
   */
  public static final double MAX_LINEAR_SPEED =
      DRIVE_MOTOR.freeSpeedRps() / DRIVE_GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS_METERS;

  // Default driver acceleration limits, m/s^2. Tunable at runtime under /Tuning/Drive/. Liberal on
  // purpose: they only clip sudden stick slams. Deceleration is higher so the driver can always
  // stop quickly.
  public static final double DRIVER_MAX_ACCEL = 10.0;
  public static final double DRIVER_MAX_DECEL = 15.0;

  // Driver acceleration and deceleration cap while the arm is away from its hard stops, m/s^2.
  // A hard start or stop swings a raised arm, which can knock it off its goal and, on a tall
  // robot, help tip it. This applies to slowing down too, so stops take longer with the arm up.
  public static final double ARM_RAISED_MAX_ACCEL = 4.0;

  // DriveToPose limits and tolerances. Gentler than the driver's limits: it's for lining up, not
  // racing, and a smooth arrival settles faster than an aggressive one.
  public static final double ALIGN_MAX_VELOCITY = 3.0; // m/s
  public static final double ALIGN_MAX_ACCEL = 4.0; // m/s^2
  public static final double ALIGN_MAX_ANGULAR_VELOCITY = 6.0; // rad/s
  public static final double ALIGN_MAX_ANGULAR_ACCEL = 12.0; // rad/s^2
  public static final double ALIGN_DISTANCE_TOLERANCE = 0.02; // m
  public static final double ALIGN_ANGLE_TOLERANCE = Math.toRadians(2.0);
  // Also has to be nearly stopped (see AlignTargets.isAligned).
  public static final double ALIGN_SETTLED_SPEED = 0.05; // m/s

  /**
   * Front to back over the bumpers, meters. A square 27.5 in frame (modules 11.5 in from center)
   * plus about 3.5 in of bumper on each end. Measure the real robot.
   */
  public static final double BUMPER_LENGTH_METERS = Units.inchesToMeters(35.0);

  /**
   * The robot's moment of inertia about its center, kg*m^2: how hard it is to spin up, the turning
   * version of mass. Estimated by treating the robot as a solid square slab the size of its
   * bumpers: mass * (length^2 + width^2) / 12. A real robot with its weight near the middle spins
   * more easily, so this is on the high side. The simulator uses it; a SysId test that spins the
   * robot in place would measure it.
   */
  public static final double ROBOT_MOI_KG_M2 =
      Constants.ROBOT_MASS_KG * 2 * BUMPER_LENGTH_METERS * BUMPER_LENGTH_METERS / 12.0;

  /** Rad/s, assuming all wheels drive tangentially at max linear speed. */
  public static final double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS;

  private static Translation2d[] moduleTranslations() {
    var translations = new Translation2d[MODULES.length];
    for (int i = 0; i < MODULES.length; i++) {
      translations[i] = MODULES[i].location();
    }
    return translations;
  }

  private static double driveBaseRadius() {
    double radius = 0.0;
    for (var translation : MODULE_TRANSLATIONS) {
      radius = Math.max(radius, translation.getNorm());
    }
    return radius;
  }
}
