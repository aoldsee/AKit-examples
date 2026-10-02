package first.robot.subsystems.vision;

import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.util.Units;

public final class VisionConstants {
  private VisionConstants() {}

  /** NetworkTables names, which must match the names set in each Limelight's web UI. */
  public static final String[] CAMERA_NAMES = {"limelight-front", "limelight-back"};

  /**
   * Where each camera sits on the robot: meters forward, left, and up from the robot's center on
   * the floor, then its rotation. VisionIOLimelight sends this to each camera at startup, so it
   * overrides whatever the web UI says. Rotation3d pitch is positive nose-down, so a camera tilted
   * up has a negative pitch.
   */
  public static final Transform3d[] ROBOT_TO_CAMERA = {
    new Transform3d(0.25, 0.0, 0.3, new Rotation3d(0.0, Units.degreesToRadians(-20.0), 0.0)),
    new Transform3d(-0.25, 0.0, 0.3, new Rotation3d(0.0, Units.degreesToRadians(-20.0), Math.PI))
  };

  // Tags farther than this are only a few pixels wide, so small detection errors become large
  // position errors. Raise it if the robot needs vision from across the field.
  public static final double MAX_TAG_DISTANCE_METERS = 6.0;

  // How old the heading MegaTag2 solves with is, in seconds: up to one 20 ms loop since the robot
  // measured it, plus network delay. While the robot spins, that heading is behind by spin rate *
  // this delay, and a heading off by some angle puts the position off by about distance * angle
  // (in radians). So Vision adds distance * spin rate * this delay to the standard deviation, and
  // trusts an estimate less the faster the robot spins. Raise it if vision drags the pose around
  // while spinning in front of tags; 0 turns the scaling off.
  public static final double MEGATAG2_HEADING_DELAY_SECS = 0.02;

  // About 1.5 turns per second. Past this, the stale heading is so far off that even a small
  // amount of trust would drag the pose, so the estimate is dropped outright.
  public static final double MAX_YAW_VELOCITY_RAD_PER_SEC = Math.toRadians(540.0);

  // Observations whose height is further than this from the floor are wrong and get dropped.
  public static final double MAX_Z_ERROR_METERS = 0.75;

  /**
   * How much to trust one tag seen from 1 meter away, as a standard deviation in meters. Vision
   * scales it up with distance squared and down with the number of tags: a typical view of 5 tags
   * at 4 m works out to about 1 cm. This is already the MegaTag2 figure, which is steadier than a
   * full 3D solve. LimelightSim adds exactly this much noise, so in sim the trust is always right.
   * On a real robot, estimate it by parking in front of some tags and measuring how much the
   * accepted poses wander.
   */
  public static final double LINEAR_STD_DEV_BASELINE = 0.0025;

  /** Per-camera multipliers on the standard deviation, to trust some cameras more than others. */
  public static final double[] CAMERA_STD_DEV_FACTORS = {1.0, 1.0};
}
