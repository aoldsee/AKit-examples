package first.robot.subsystems.arm;

import first.robot.util.MotorFaults;
import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  @AutoLog
  class ArmIOInputs {
    public boolean motorConnected = false;
    public boolean encoderConnected = false;

    /** Fused motor and CANcoder position, what the controller closes on. */
    public double positionRad = 0.0;

    /** Raw CANcoder reading, for checking the fused value against. */
    public double absolutePositionRad = 0.0;

    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public MotorFaults motorFaults = MotorFaults.NONE;
  }

  /**
   * The gains that can change at runtime. Volts per rotation of error for kP, volts per rotation/s
   * of error for kD, volts for kG, arm rotations/s and rotations/s^2 for the Motion Magic limits.
   */
  record Gains(double kP, double kD, double kG, double cruiseVelocity, double acceleration) {}

  default void updateInputs(ArmIOInputs inputs) {}

  /** Sends new gains to the motor controller. Called only when a value changes. */
  default void setGains(Gains gains) {}

  /** Profiled move to an arm angle. Gravity feedforward is applied by the controller. */
  default void setPosition(double angleRad) {}

  /** Open loop, no gravity compensation. */
  default void setVoltage(double volts) {}
}
