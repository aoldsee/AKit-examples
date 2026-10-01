package first.robot.subsystems.intake;

import first.robot.util.MotorFaults;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  class IntakeIOInputs {
    public boolean motorConnected = false;

    /** Raw beam break reading, already corrected for SENSOR_INVERTED. Not debounced. */
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
