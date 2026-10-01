package first.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.TreeSet;
import java.util.function.Supplier;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.util.Units;
import org.wpilib.networktables.DoubleArrayPublisher;
import org.wpilib.networktables.DoubleArraySubscriber;
import org.wpilib.networktables.DoubleSubscriber;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.system.RobotController;

/**
 * Reads a Limelight's MegaTag2 pose estimates from NetworkTables. Limelight needs no vendor
 * library; it publishes everything to NT. In SIM, {@link LimelightSim} publishes the same topics so
 * this class runs unchanged.
 *
 * <p>MegaTag2 asks the robot for its heading (from the gyro) and solves only for position, which is
 * far steadier than solving all six axes from the image. The tradeoff is that it can't correct the
 * heading.
 */
public class VisionIOLimelight implements VisionIO {
  // botpose array layout: x, y, z, roll, pitch, yaw, latency ms, tag count, tag span, average
  // distance, average area, then 7 values per tag starting with its ID.
  private static final int LATENCY_INDEX = 6;
  private static final int TAG_COUNT_INDEX = 7;
  private static final int AVERAGE_DISTANCE_INDEX = 9;
  private static final int FIRST_TAG_INDEX = 11;
  private static final int VALUES_PER_TAG = 7;

  private final Supplier<Rotation2d> headingSupplier;
  private final DoubleArrayPublisher cameraPosePublisher;
  private final DoubleArrayPublisher orientationPublisher;
  private final DoubleSubscriber latencySubscriber;
  private final DoubleSubscriber txSubscriber;
  private final DoubleSubscriber tySubscriber;
  private final DoubleArraySubscriber megatag2Subscriber;

  /**
   * @param name the Limelight's name from its web UI, which is also its NT table
   * @param robotToCamera where the camera sits on the robot; sent to the camera at startup
   * @param headingSupplier the robot's current heading estimate, which MegaTag2 needs
   */
  public VisionIOLimelight(
      String name, Transform3d robotToCamera, Supplier<Rotation2d> headingSupplier) {
    this.headingSupplier = headingSupplier;
    var table = NetworkTableInstance.getDefault().getTable(name);

    // Tell the camera where it's mounted, instead of typing the same numbers into its web UI. The
    // code is then the one place the camera's position lives, so the simulator, the robot, and the
    // camera can't disagree, and moving a camera is a code change that shows up in git.
    // NetworkTables keeps a published value and delivers it whenever a subscriber connects, so
    // sending it once here still reaches a camera that boots after the robot code. The camera
    // doesn't save it; it uses the pushed pose until it reboots, and the robot code pushes it again
    // on its next start.
    cameraPosePublisher = table.getDoubleArrayTopic("camerapose_robotspace_set").publish();
    cameraPosePublisher.set(toLimelightCameraPose(robotToCamera));

    orientationPublisher = table.getDoubleArrayTopic("robot_orientation_set").publish();
    latencySubscriber = table.getDoubleTopic("tl").subscribe(0.0);
    txSubscriber = table.getDoubleTopic("tx").subscribe(0.0);
    tySubscriber = table.getDoubleTopic("ty").subscribe(0.0);
    megatag2Subscriber =
        table.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[] {});
  }

  /**
   * A WPILib camera transform in the array layout Limelight expects: forward, left, and up in
   * meters, then roll, pitch, and yaw in degrees.
   *
   * <p>The axes match WPILib (+x forward, +y left, +z up), but Limelight measures pitch the other
   * way: positive tilts the camera up, where WPILib's positive pitch tilts it down. So pitch flips
   * sign. Roll and yaw don't. Package-private for the unit tests.
   */
  static double[] toLimelightCameraPose(Transform3d robotToCamera) {
    var rotation = robotToCamera.getRotation();
    return new double[] {
      robotToCamera.getX(),
      robotToCamera.getY(),
      robotToCamera.getZ(),
      Units.radiansToDegrees(rotation.getX()),
      -Units.radiansToDegrees(rotation.getY()),
      Units.radiansToDegrees(rotation.getZ())
    };
  }

  /** The reverse of {@link #toLimelightCameraPose}, for the simulated Limelight. */
  static Transform3d fromLimelightCameraPose(double[] values) {
    return new Transform3d(
        values[0],
        values[1],
        values[2],
        new Rotation3d(
            Units.degreesToRadians(values[3]),
            -Units.degreesToRadians(values[4]),
            Units.degreesToRadians(values[5])));
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    // The Limelight publishes latency every frame, so a stale value means it's gone. 2027 NT and
    // RobotController times are in nanoseconds (2026 used microseconds).
    long nanosSinceUpdate = RobotController.getMonotonicTime() - latencySubscriber.getLastChange();
    inputs.connected = nanosSinceUpdate < 250_000_000L;

    inputs.latestTargetObservation =
        new TargetObservation(
            Rotation2d.fromDegrees(txSubscriber.get()), Rotation2d.fromDegrees(tySubscriber.get()));

    orientationPublisher.accept(
        new double[] {headingSupplier.get().getDegrees(), 0.0, 0.0, 0.0, 0.0, 0.0});
    // Sends the heading now instead of at the next NT update, as Limelight recommends.
    NetworkTableInstance.getDefault().flush();

    Set<Integer> tagIds = new TreeSet<>();
    List<PoseObservation> observations = new ArrayList<>();
    // readQueue returns every frame since the last call, not just the newest, so no estimate is
    // lost when the camera runs faster than the 50 Hz robot loop.
    for (var sample : megatag2Subscriber.readQueue()) {
      double[] values = sample.value;
      if (values.length < FIRST_TAG_INDEX) {
        continue;
      }
      for (int i = FIRST_TAG_INDEX; i < values.length; i += VALUES_PER_TAG) {
        tagIds.add((int) values[i]);
      }
      observations.add(
          new PoseObservation(
              // The time NT received the frame (nanoseconds), minus how long the camera took to
              // produce it (milliseconds).
              sample.timestamp * 1.0e-9 - values[LATENCY_INDEX] * 1.0e-3,
              new Pose3d(
                  values[0],
                  values[1],
                  values[2],
                  new Rotation3d(
                      Units.degreesToRadians(values[3]),
                      Units.degreesToRadians(values[4]),
                      Units.degreesToRadians(values[5]))),
              (int) values[TAG_COUNT_INDEX],
              values[AVERAGE_DISTANCE_INDEX]));
    }

    inputs.poseObservations = observations.toArray(new PoseObservation[0]);
    inputs.tagIds = tagIds.stream().mapToInt(Integer::intValue).toArray();
  }
}
