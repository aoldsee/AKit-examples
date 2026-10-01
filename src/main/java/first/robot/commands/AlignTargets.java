package first.robot.commands;

import first.robot.subsystems.drive.DriveConstants;
import first.robot.util.FieldGeometry;
import java.util.Optional;
import org.wpilib.fields.FieldTag;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.kinematics.ChassisVelocities;

/** Field poses worth driving to, worked out from where the AprilTags are. */
public final class AlignTargets {
  private AlignTargets() {}

  /** How far in front of a tag to stop, measured from the tag to the robot's center, meters. */
  public static final double STANDOFF_METERS = 1.0;

  // Tags higher than this are on structures the robot can't drive up to.
  private static final double MAX_TAG_HEIGHT_METERS = 1.5;
  // Keep targets this far from the field walls so the robot fits.
  private static final double WALL_MARGIN_METERS = 0.5;

  /**
   * The pose {@code standoffMeters} in front of a tag, facing it. A tag's pose points out of its
   * printed face, so "in front" is along the tag's own x axis; turning 180 degrees faces the robot
   * back at it.
   */
  public static Pose2d inFrontOf(FieldTag tag, double standoffMeters) {
    return tag.getPose()
        .toPose2d()
        .transformBy(new Transform2d(standoffMeters, 0.0, Rotation2d.k180deg));
  }

  /**
   * Close enough to the target, pointed the right way, and nearly stopped. DriveToPose finishes on
   * this, and the driver's controller rumbles on it, so both always agree on what "lined up" means.
   * Without the speed check, a robot passing through the target at speed would count as arrived.
   */
  public static boolean isAligned(Pose2d robot, ChassisVelocities velocities, Pose2d target) {
    double distance = robot.getTranslation().getDistance(target.getTranslation());
    double angleError = target.getRotation().minus(robot.getRotation()).getRadians();
    double speed = Math.hypot(velocities.vx, velocities.vy);
    return distance < DriveConstants.ALIGN_DISTANCE_TOLERANCE
        && Math.abs(angleError) < DriveConstants.ALIGN_ANGLE_TOLERANCE
        && speed < DriveConstants.ALIGN_SETTLED_SPEED;
  }

  /** The alignment pose for the nearest tag the robot can actually stand in front of. */
  public static Optional<Pose2d> nearestTag(Pose2d robot) {
    Pose2d best = null;
    double bestDistance = Double.POSITIVE_INFINITY;
    for (var tag : FieldGeometry.FIELD.getTags()) {
      if (tag.getPose().getZ() > MAX_TAG_HEIGHT_METERS) {
        continue;
      }
      var candidate = inFrontOf(tag, STANDOFF_METERS);
      if (!onField(candidate)) {
        continue;
      }
      double distance = candidate.getTranslation().getDistance(robot.getTranslation());
      if (distance < bestDistance) {
        best = candidate;
        bestDistance = distance;
      }
    }
    return Optional.ofNullable(best);
  }

  private static boolean onField(Pose2d pose) {
    return pose.getX() > WALL_MARGIN_METERS
        && pose.getX() < FieldGeometry.FIELD.getFieldLength() - WALL_MARGIN_METERS
        && pose.getY() > WALL_MARGIN_METERS
        && pose.getY() < FieldGeometry.FIELD.getFieldWidth() - WALL_MARGIN_METERS;
  }
}
