package first.robot.subsystems.intake;

import first.robot.util.MotorFaults;
import org.littletonrobotics.junction.AutoLog;

/**
 * The intake's IO layer: the thin part that talks to hardware. Inputs holds what the sensors read,
 * and AdvantageKit logs it every loop. The methods send commands to the motor; each has an empty
 * default, so an IO that does nothing is just {@code new IntakeIO() {}}.
 */
public interface IntakeIO {
  /** Everything read from the hardware. @AutoLog generates IntakeIOInputsAutoLogged from this. */
  @AutoLog
  class IntakeIOInputs {
    public boolean motorConnected = false;

    /** True while the beam break sees a piece (see IntakeConstants.BEAM_BREAK). Not debounced. */
    public boolean sensorBlocked = false;

    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public MotorFaults motorFaults = MotorFaults.NONE;
  }

  default void updateInputs(IntakeIOInputs inputs) {}

  /** Positive pulls a piece in. */
  default void setVoltage(double volts) {}
}
