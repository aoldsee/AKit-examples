package first.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;

public interface VisionIO {
  @AutoLog
  class VisionIOInputs {
    public boolean connected = false;

    /** Angle to the best tag, for simple "turn toward the target" control. Not used for pose. */
    public TargetObservation latestTargetObservation =
        new TargetObservation(Rotation2d.ZERO, Rotation2d.ZERO);

    /** Every pose estimate since the last loop. Usually zero or one, sometimes more. */
    public PoseObservation[] poseObservations = new PoseObservation[0];

    public int[] tagIds = new int[0];
  }

  /** Horizontal and vertical angle to a target, in the camera's own sign convention. */
  record TargetObservation(Rotation2d tx, Rotation2d ty) {}

  /**
   * One robot pose estimate from a camera.
   *
   * @param timestamp when the image was taken, in the same timebase as the pose estimator
   */
  record PoseObservation(double timestamp, Pose3d pose, int tagCount, double averageTagDistance) {}

  default void updateInputs(VisionIOInputs inputs) {}
}
