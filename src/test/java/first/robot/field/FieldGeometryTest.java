package first.robot.field;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import first.robot.subsystems.drive.DriveConstants;
import org.junit.jupiter.api.Test;
import org.wpilib.hardware.hal.AllianceStationID;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.simulation.DriverStationSim;

/** Field math that every alliance-dependent feature leans on. */
class FieldGeometryTest {
  @Test
  void flipTurnsTheFieldAroundItsCenter() {
    var blue = new Pose2d(1.0, 2.0, Rotation2d.fromDegrees(30.0));
    var red = FieldGeometry.flip(blue);

    assertEquals(FieldGeometry.LENGTH_METERS - 1.0, red.getX(), 1e-9);
    assertEquals(FieldGeometry.WIDTH_METERS - 2.0, red.getY(), 1e-9);
    assertEquals(-150.0, red.getRotation().getDegrees(), 1e-9);
    // Flipping twice gets back where it started.
    var back = FieldGeometry.flip(red);
    assertEquals(blue.getX(), back.getX(), 1e-9);
    assertEquals(blue.getY(), back.getY(), 1e-9);
    assertEquals(30.0, back.getRotation().getDegrees(), 1e-9);
  }

  @Test
  void flippedBlueHubLandsOnTheRedHub() {
    // Red's hub tags are 2 to 5 and 8 to 11. Their average should be where the flip puts blue's.
    var sum = Translation2d.ZERO;
    for (int id : new int[] {2, 3, 4, 5, 8, 9, 10, 11}) {
      sum = sum.plus(FieldGeometry.FIELD.getTagPose(id).orElseThrow().toPose2d().getTranslation());
    }
    var redHub = sum.div(8);
    var flipped = FieldGeometry.flip(FieldGeometry.BLUE_HUB_CENTER);
    assertEquals(redHub.getX(), flipped.getX(), 0.01);
    assertEquals(redHub.getY(), flipped.getY(), 0.01);
  }

  @Test
  void startingPosesAreAgainstTheWallFacingDownfield() {
    double previousY = Double.POSITIVE_INFINITY;
    for (int station = 1; station <= FieldGeometry.STATION_COUNT; station++) {
      var pose = FieldGeometry.startingPose(station);
      assertEquals(DriveConstants.BUMPER_LENGTH_METERS / 2, pose.getX(), 1e-9);
      assertEquals(0.0, pose.getRotation().getDegrees(), 1e-9);
      assertTrue(pose.getY() > 0.0 && pose.getY() < FieldGeometry.WIDTH_METERS);
      // Station 1 is at the drivers' left (+y), then 2, then 3.
      assertTrue(pose.getY() < previousY);
      previousY = pose.getY();
    }
    assertEquals(FieldGeometry.WIDTH_METERS / 2, FieldGeometry.startingPose(2).getY(), 1e-9);
    assertThrows(IllegalArgumentException.class, () -> FieldGeometry.startingPose(4));
  }

  @Test
  void headingTowardPointsAtTheTarget() {
    var robot = new Pose2d(1.0, 1.0, Rotation2d.ZERO);
    assertEquals(
        90.0, FieldGeometry.headingToward(robot, new Translation2d(1.0, 5.0)).getDegrees(), 1e-9);
    assertEquals(
        180.0,
        Math.abs(FieldGeometry.headingToward(robot, new Translation2d(-2.0, 1.0)).getDegrees()),
        1e-9);
    // On top of the point there's no direction, so the robot keeps its heading.
    var onPoint = new Pose2d(3.0, 3.0, Rotation2d.fromDegrees(42.0));
    assertEquals(
        42.0, FieldGeometry.headingToward(onPoint, new Translation2d(3.0, 3.0)).getDegrees(), 1e-9);
  }

  @Test
  void downfieldFacesAwayFromOurDrivers() {
    HAL.initialize();
    DriverStationSim.setAllianceStationId(AllianceStationID.BLUE_1);
    DriverStationSim.notifyNewData();
    assertEquals(0.0, FieldGeometry.downfield().getDegrees(), 1e-9);

    DriverStationSim.setAllianceStationId(AllianceStationID.RED_1);
    DriverStationSim.notifyNewData();
    assertEquals(180.0, Math.abs(FieldGeometry.downfield().getDegrees()), 1e-9);
  }
}
