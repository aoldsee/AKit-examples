package first.robot.field;

import first.robot.subsystems.drive.DriveConstants;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchState;
import org.wpilib.fields.Field;
import org.wpilib.fields.Fields;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;

/**
 * Where things are on the field, and how to turn "our side" positions into real field positions.
 *
 * <p>Field coordinates always have the origin at the blue alliance wall's right corner (as seen by
 * the blue drivers), +x pointing toward red, +y to the blue drivers' left. That doesn't change with
 * the alliance: a red robot at its own wall has an x near the field length. So positions here are
 * written once, from blue's side, and {@link #flipIfRed} moves them to red's side when needed.
 *
 * <p>Everything that depends on where the origin is lives in this file. If a future season moves
 * the origin (to field center, say), this is the file to change.
 */
public final class FieldGeometry {
  private FieldGeometry() {}

  /**
   * The field and its AprilTags. 2027 moved tag layouts from AprilTagFieldLayout to the Field
   * class.
   */
  public static final Field FIELD = Field.loadField(Fields.FRC_2026_REBUILT_WELDED);

  public static final double LENGTH_METERS = FIELD.getFieldLength();
  public static final double WIDTH_METERS = FIELD.getFieldWidth();

  // The eight tags on the faces of the blue hub. Red's hub is the same shape on the other side.
  private static final int[] BLUE_HUB_TAG_IDS = {18, 19, 20, 21, 24, 25, 26, 27};

  /** Center of the blue hub: the average of its tags' positions, which surround it evenly. */
  public static final Translation2d BLUE_HUB_CENTER = averageTagPosition(BLUE_HUB_TAG_IDS);

  /** Number of driver stations per alliance. */
  public static final int STATION_COUNT = 3;

  /**
   * Starting pose with the back bumper against the alliance wall, centered on a driver station (1,
   * 2, or 3), facing downfield. Blue side; use {@link #flipIfRed} for the real pose.
   *
   * <p>The stations are assumed to split the wall evenly, with station 1 at the drivers' left (the
   * +y end). Check both against the game manual's field drawings before trusting them on a real
   * field.
   */
  public static Pose2d startingPose(int station) {
    if (station < 1 || station > STATION_COUNT) {
      throw new IllegalArgumentException("No driver station " + station);
    }
    double stationWidth = WIDTH_METERS / STATION_COUNT;
    double y = WIDTH_METERS - (station - 0.5) * stationWidth;
    return new Pose2d(DriveConstants.BUMPER_LENGTH_METERS / 2, y, Rotation2d.ZERO);
  }

  /** True when the Driver Station says we're red. Unknown counts as blue. */
  public static boolean isRed() {
    return MatchState.getAlliance().orElse(Alliance.BLUE) == Alliance.RED;
  }

  /**
   * The heading that faces away from our drivers, toward the other alliance: 0 degrees on blue, 180
   * on red.
   */
  public static Rotation2d downfield() {
    return isRed() ? Rotation2d.k180deg : Rotation2d.ZERO;
  }

  /**
   * The same spot on the other alliance's side. The 2026 field is rotationally symmetric: red's
   * half is blue's half turned 180 degrees about the field center, so x, y, and heading all flip.
   * (Some years mirror the field left to right instead. Then only x and heading flip.)
   */
  public static Pose2d flip(Pose2d pose) {
    return new Pose2d(flip(pose.getTranslation()), pose.getRotation().plus(Rotation2d.k180deg));
  }

  public static Translation2d flip(Translation2d translation) {
    return new Translation2d(LENGTH_METERS - translation.getX(), WIDTH_METERS - translation.getY());
  }

  /** A blue-side pose moved to our side of the field. */
  public static Pose2d flipIfRed(Pose2d bluePose) {
    return isRed() ? flip(bluePose) : bluePose;
  }

  /** A blue-side point moved to our side of the field. */
  public static Translation2d flipIfRed(Translation2d bluePoint) {
    return isRed() ? flip(bluePoint) : bluePoint;
  }

  /** The heading that points the robot's front from {@code robot} at {@code point}. */
  public static Rotation2d headingToward(Pose2d robot, Translation2d point) {
    var offset = point.minus(robot.getTranslation());
    // Standing exactly on the point there's no direction to face, so keep the current heading.
    return offset.getAngle().orElse(robot.getRotation());
  }

  private static Translation2d averageTagPosition(int[] ids) {
    var sum = Translation2d.ZERO;
    for (int id : ids) {
      var tag =
          FIELD
              .getTagPose(id)
              .orElseThrow(() -> new IllegalStateException("Field has no tag " + id));
      sum = sum.plus(tag.toPose2d().getTranslation());
    }
    return sum.div(ids.length);
  }
}
