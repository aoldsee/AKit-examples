package first.robot.subsystems.arm;

import first.robot.util.MotorFaults;
import org.littletonrobotics.junction.AutoLog;

/** The arm's IO layer, built the same way as {@link first.robot.subsystems.intake.IntakeIO}. */
public interface ArmIO {
  @AutoLog
  class ArmIOInputs {
    public boolean motorConnected = false;
    public boolean encoderConnected = false;

    /**
     * Arm angle. The Talon combines ("fuses") its own motor sensor with the CANcoder on the arm
     * shaft into this one reading, and it's the angle the Talon's position control uses.
     */
    public double positionRad = 0.0;

    /** Raw CANcoder reading, for checking the fused value against. */
    public double absolutePositionRad = 0.0;

    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public MotorFaults motorFaults = MotorFaults.NONE;
  }

  /**
   * The gains that can change at runtime. Phoenix works in rotations, not radians, so these do too
   * (ArmIOTalonFX converts the angles). Volts per rotation of error for kP, volts per rotation/s of
   * error for kD, volts for kG, and arm rotations/s and rotations/s^2 for cruise velocity and
   * acceleration (the limits of the Talon's motion profile; see ArmIOTalonFX).
   */
  record Gains(double kP, double kD, double kG, double cruiseVelocity, double acceleration) {}

  default void updateInputs(ArmIOInputs inputs) {}

  /** Sends new gains to the motor controller. Called only when a value changes. */
  default void setGains(Gains gains) {}

  /**
   * Moves the arm to an angle. The control loop runs on the motor controller (the Talon), not in
   * robot code: it plans a smooth move, then runs PID plus a gravity feedforward to follow it.
   */
  default void setPosition(double angleRad) {}

  /** Open loop, no gravity compensation. */
  default void setVoltage(double volts) {}
}
