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

  // How far apart in time the heading MegaTag2 solves with and the moment the picture was taken
  // can be, in seconds. The robot sends its heading once per 20 ms loop, so the camera solves with
  // one that's up to a loop older or newer than its picture, plus network delay. While the robot
  // spins, that heading is off by spin rate * this delay, and a heading off by some angle puts the
  // position off by about distance * angle (in radians). So Vision adds distance * spin rate * this
  // delay to the standard deviation, and
  // trusts an estimate less the faster the robot spins. Raise it if vision drags the pose around
  // while spinning in front of tags; 0 turns the scaling off.
  public static final double MEGATAG2_HEADING_DELAY_SECS = 0.02;

  // About 1.5 turns per second. Past this, the mismatched heading is so far off that even a small
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

  /**
   * How much to trust MegaTag1's heading from one tag seen from 1 meter away, as a standard
   * deviation in radians. Scaled the same way as LINEAR_STD_DEV_BASELINE: a typical view of 5 tags
   * at 4 m works out to about 2 degrees. Each frame then moves the heading only about 3% of the way
   * (see DriveConstants.ODOMETRY_STD_DEVS), so the gyro's smooth heading still dominates from
   * moment to moment, but a heading that's wrong (a crooked start, gyro drift) is pulled back
   * within a second or two of seeing tags. An estimate; measure it the same way as the linear one,
   * watching the heading instead.
   */
  public static final double ANGULAR_STD_DEV_BASELINE = 0.01;

  // MegaTag1 needs at least this many tags for its heading. From a single flat tag, two quite
  // different camera angles can produce nearly the same image, so its heading can flip.
  public static final int MEGATAG_1_MIN_TAGS = 2;

  /** Per-camera multipliers on the standard deviation, to trust some cameras more than others. */
  public static final double[] CAMERA_STD_DEV_FACTORS = {1.0, 1.0};
}
