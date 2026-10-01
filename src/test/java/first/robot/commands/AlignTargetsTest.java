package first.robot.commands;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import first.robot.subsystems.drive.DriveConstants;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.ChassisVelocities;

/** What counts as "lined up", shared by DriveToPose and the driver's rumble. */
class AlignTargetsTest {
  private static final Pose2d TARGET = new Pose2d(3.0, 4.0, Rotation2d.fromDegrees(90.0));
  private static final ChassisVelocities STOPPED = new ChassisVelocities();

  @Test
  void onTargetAndStoppedIsAligned() {
    assertTrue(AlignTargets.isAligned(TARGET, STOPPED, TARGET));
    var slightlyOff = new Pose2d(3.01, 4.0, Rotation2d.fromDegrees(91.0));
    assertTrue(AlignTargets.isAligned(slightlyOff, STOPPED, TARGET));
  }

  @Test
  void eachToleranceMatters() {
    double tooFar = DriveConstants.ALIGN_DISTANCE_TOLERANCE * 1.5;
    assertFalse(
        AlignTargets.isAligned(
            new Pose2d(3.0 + tooFar, 4.0, TARGET.getRotation()), STOPPED, TARGET));
    assertFalse(
        AlignTargets.isAligned(
            new Pose2d(3.0, 4.0, Rotation2d.fromDegrees(95.0)), STOPPED, TARGET));
    // Right on target but still moving: it would coast past, so it isn't there yet.
    var moving = new ChassisVelocities(DriveConstants.ALIGN_SETTLED_SPEED * 2, 0.0, 0.0);
    assertFalse(AlignTargets.isAligned(TARGET, moving, TARGET));
  }
}
