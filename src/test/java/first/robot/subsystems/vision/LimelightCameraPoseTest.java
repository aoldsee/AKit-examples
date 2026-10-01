package first.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.util.Units;

/** The camera pose sent to the Limelight, which uses a different sign for pitch than WPILib. */
class LimelightCameraPoseTest {
  @Test
  void cameraTiltedUpHasPositiveLimelightPitch() {
    // 0.3 m forward, 0.1 m left, 0.5 m up, tilted 20 degrees up, turned 30 degrees left.
    var robotToCamera =
        new Transform3d(
            0.3,
            0.1,
            0.5,
            new Rotation3d(0.0, Units.degreesToRadians(-20.0), Units.degreesToRadians(30.0)));

    assertArrayEquals(
        new double[] {0.3, 0.1, 0.5, 0.0, 20.0, 30.0},
        VisionIOLimelight.toLimelightCameraPose(robotToCamera),
        1e-9);
  }

  @Test
  void convertingThereAndBackGivesTheSameCamera() {
    for (var robotToCamera : VisionConstants.ROBOT_TO_CAMERA) {
      var back =
          VisionIOLimelight.fromLimelightCameraPose(
              VisionIOLimelight.toLimelightCameraPose(robotToCamera));
      assertEquals(0.0, back.getTranslation().getDistance(robotToCamera.getTranslation()), 1e-9);
      assertEquals(robotToCamera.getRotation().getX(), back.getRotation().getX(), 1e-9);
      assertEquals(robotToCamera.getRotation().getY(), back.getRotation().getY(), 1e-9);
      assertEquals(robotToCamera.getRotation().getZ(), back.getRotation().getZ(), 1e-9);
    }
  }
}
