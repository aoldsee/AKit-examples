package first.robot.subsystems.drive;

import first.robot.util.MotorFaults;
import org.littletonrobotics.junction.AutoLog;
import org.wpilib.math.geometry.Rotation2d;

public interface ModuleIO {
  @AutoLog
  class ModuleIOInputs {
    public boolean driveConnected = false;

    /** Wheel radians (after the drive reduction). */
    public double drivePositionRad = 0.0;

    public double driveVelocityRadPerSec = 0.0;
    public double driveAppliedVolts = 0.0;
    public double driveCurrentAmps = 0.0;
    public MotorFaults driveFaults = MotorFaults.NONE;

    public boolean turnConnected = false;
    public boolean turnEncoderConnected = false;

    /** Raw CANcoder reading. turnPosition is what the controller actually closes on. */
    public Rotation2d turnAbsolutePosition = Rotation2d.ZERO;

    public Rotation2d turnPosition = Rotation2d.ZERO;
    public double turnVelocityRadPerSec = 0.0;
    public double turnAppliedVolts = 0.0;
    public double turnCurrentAmps = 0.0;
    public MotorFaults turnFaults = MotorFaults.NONE;

    /** High-rate samples since the last cycle. All three arrays share indices. */
    public double[] odometryTimestamps = new double[] {};

    public double[] odometryDrivePositionsRad = new double[] {};
    public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};
  }

  default void updateInputs(ModuleIOInputs inputs) {}

  /** Volts or amps, matching the module's closed-loop output type. */
  default void setDriveOpenLoop(double output) {}

  /** Volts or amps, matching the module's closed-loop output type. */
  default void setTurnOpenLoop(double output) {}

  /** Wheel rad/s. */
  default void setDriveVelocity(double velocityRadPerSec) {}

  default void setTurnPosition(Rotation2d rotation) {}
}
