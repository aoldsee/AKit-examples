package first.robot.subsystems.intake;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;
import first.robot.hardware.Motors;
import first.robot.subsystems.arm.ArmConstants;
import org.wpilib.math.util.Units;

/**
 * Placeholder intake: rollers on the end of the arm, driven by one Kraken X44, with a beam break
 * that sees when a game piece is all the way in. Like the coral end effectors many teams built in
 * 2025.
 */
public final class IntakeConstants {
  private IntakeConstants() {}

  // Rides on the arm, so its wiring runs along the arm's and joins the same CAN bus.
  public static final CANBus CAN_BUS = ArmConstants.CAN_BUS;
  public static final int MOTOR_ID = 16;

  /** Which spin direction counts as positive. Positive must pull a piece in. */
  public static final InvertedValue MOTOR_DIRECTION = InvertedValue.CounterClockwise_Positive;

  /** SystemCore digital input port the beam break is wired to. */
  public static final int SENSOR_CHANNEL = 0;

  /** What a beam break's signal means. Depends on the sensor and how it's wired. */
  public enum BeamBreak {
    READS_TRUE_WHEN_BLOCKED,
    READS_TRUE_WHEN_CLEAR
  }

  /**
   * Many beam breaks read true while the beam is clear. To check a real one, put a piece in by hand
   * and watch Intake/SensorBlocked in the log: it should read true. If not, switch this.
   */
  public static final BeamBreak BEAM_BREAK = BeamBreak.READS_TRUE_WHEN_CLEAR;

  /**
   * Seconds the sensor must agree before Intake believes it. A piece bouncing in the rollers can
   * flicker the beam for a few milliseconds, which would otherwise end intake() early.
   */
  public static final double SENSOR_DEBOUNCE_SECONDS = 0.04;

  public static final Motors.Spec MOTOR = Motors.KRAKEN_X44_FOC;

  /** Motor rotations per roller rotation. */
  public static final double GEAR_RATIO = 3.0;

  /** kg*m^2. Rollers are light; a 2 in, 0.3 kg roller assembly is about this. */
  public static final double ROLLER_MOI = 0.5 * 0.3 * Math.pow(Units.inchesToMeters(1.0), 2);

  // Volts. Positive pulls a piece in. Ejecting is slower than intaking, so the piece leaves where
  // the robot is pointed instead of flying off.
  public static final double INTAKE_VOLTS = 8.0;
  public static final double EJECT_VOLTS = -6.0;

  /**
   * Volts to keep a held piece from sliding out. The rollers are stalled against the piece, so
   * nearly all of this turns into current and heat: at 1 V an X44 draws about 25 A from the motor's
   * side but only about 2 A from the battery. Higher holds tighter and runs hotter.
   */
  public static final double HOLD_VOLTS = 1.0;

  /** Keep ejecting this long after the sensor clears, so the piece is fully out of the rollers. */
  public static final double EJECT_FOLLOW_THROUGH_SECONDS = 0.2;

  public static final double STATOR_CURRENT_LIMIT_AMPS = 40.0;
}
