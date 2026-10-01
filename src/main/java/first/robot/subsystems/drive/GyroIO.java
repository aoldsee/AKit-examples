package first.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;
import org.wpilib.math.geometry.Rotation2d;

public interface GyroIO {
  @AutoLog
  class GyroIOInputs {
    public boolean connected = false;
    public Rotation2d yawPosition = Rotation2d.ZERO;
    public double yawVelocityRadPerSec = 0.0;

    /** High-rate samples since the last cycle, aligned with the module odometry samples. */
    public double[] odometryYawTimestamps = new double[] {};

    public Rotation2d[] odometryYawPositions = new Rotation2d[] {};
  }

  default void updateInputs(GyroIOInputs inputs) {}
}
