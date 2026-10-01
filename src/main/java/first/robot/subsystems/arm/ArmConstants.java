package first.robot.subsystems.arm;

import com.ctre.phoenix6.CANBus;
import first.robot.util.Motors;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.math.util.Units;
import org.wpilib.simulation.SingleJointedArmSim;

/**
 * Placeholder arm: one Kraken X60 at 60:1, a CANcoder on the arm shaft, a 0.5 m, 4 kg arm.
 *
 * <p>Angles are radians at the arm, 0 is horizontal, positive is up. Phoenix's Arm_Cosine gravity
 * feedforward assumes that same zero, so a real arm's CANcoder offset must make horizontal read 0.
 */
public final class ArmConstants {
  private ArmConstants() {}

  // Its own SystemCore CAN FD port, separate from the drivetrain's CAN_S0. Arm traffic then can't
  // compete with the 250 Hz odometry signals, and a wiring fault on one bus doesn't take out the
  // other. IDs only need to be unique per bus, but continuing after the drivetrain's 1 to 13
  // keeps them unambiguous in logs and Tuner X.
  public static final CANBus CAN_BUS = new CANBus(CANPort.CAN_S1);
  public static final int MOTOR_ID = 14;
  public static final int ENCODER_ID = 15;
  public static final boolean MOTOR_INVERTED = false;
  public static final boolean ENCODER_INVERTED = false;

  /** Rotations. Zero because there's no physical arm to measure. */
  public static final double ENCODER_OFFSET = 0.0;

  /** Motor rotations per arm rotation. */
  public static final double GEAR_RATIO = 60.0;

  public static final double LENGTH_METERS = 0.5;
  public static final double MASS_KG = 4.0;

  /** kg*m^2, treating the arm as a uniform rod pivoting at one end. */
  public static final double MOI = SingleJointedArmSim.estimateMOI(LENGTH_METERS, MASS_KG);

  /** Hard stops. The arm rests on MIN_ANGLE when stowed and at power-on. */
  public static final double MIN_ANGLE_RAD = Units.degreesToRadians(-30.0);

  public static final double MAX_ANGLE_RAD = Units.degreesToRadians(110.0);

  /** Soft limits, kept 5 degrees inside the hard stops so the controller never drives into them. */
  public static final double SOFT_MIN_ANGLE_RAD = MIN_ANGLE_RAD + Units.degreesToRadians(5.0);

  public static final double SOFT_MAX_ANGLE_RAD = MAX_ANGLE_RAD - Units.degreesToRadians(5.0);

  // Presets used by bindings and autos.
  public static final double STOWED_RAD = SOFT_MIN_ANGLE_RAD;
  public static final double HORIZONTAL_RAD = 0.0;
  public static final double VERTICAL_RAD = Units.degreesToRadians(90.0);

  /**
   * Within this of either hard stop, the arm counts as stowed against it. Wide enough to include
   * the STOWED preset, which sits at the soft limit just above the lower stop.
   */
  public static final double NEAR_STOP_MARGIN_RAD = Units.degreesToRadians(10.0);

  /** Close enough to count as arrived. */
  public static final double AT_GOAL_TOLERANCE_RAD = Units.degreesToRadians(2.0);

  public static final Motors.Spec MOTOR = Motors.KRAKEN_X60_FOC;

  // Feedforward gains, per arm rotation, worked out from the motor spec and the arm's size so sim
  // works untuned. A real arm has friction and extra mass these don't know about; characterize it.
  //
  // Holding still, a motor's voltage is current * resistance, and its current is torque / Kt. So
  // "volts for a torque at the arm" is: (arm torque / gear ratio) / Kt * R.
  private static final double VOLTS_PER_ARM_NM =
      MOTOR.resistanceOhms() / (MOTOR.torquePerAmp() * GEAR_RATIO);
  private static final double GRAVITY = 9.81;

  /** Volts to hold the arm horizontal: gravity's torque is weight times half the length. */
  public static final double KG = MASS_KG * GRAVITY * (LENGTH_METERS / 2) * VOLTS_PER_ARM_NM;

  /** Volts per arm rotation/s: the back-EMF the motor makes spinning GEAR_RATIO times faster. */
  public static final double KV = GEAR_RATIO / MOTOR.rpsPerVolt();

  /** Volts per arm rotation/s^2: torque to accelerate the arm's inertia. */
  public static final double KA = MOI * 2 * Math.PI * VOLTS_PER_ARM_NM;

  /** No friction in the model. A real arm needs a little. */
  public static final double KS = 0.0;

  /** Volts per rotation of error. 1 degree of error gives about 0.14 V. */
  public static final double KP = 50.0;

  public static final double KD = 0.0;

  /** Arm rotations/s and rotations/s^2. About 60% of free speed, so there's headroom for kG. */
  public static final double CRUISE_VELOCITY = 1.0;

  public static final double ACCELERATION = 3.0;

  /** Starting values for the gains Arm allows tuning live. */
  public static final ArmIO.Gains GAINS =
      new ArmIO.Gains(KP, KD, KG, CRUISE_VELOCITY, ACCELERATION);

  public static final double STATOR_CURRENT_LIMIT_AMPS = 60.0;
}
