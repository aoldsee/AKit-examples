package first.robot.subsystems.vision;

import first.robot.sim.SimulatedMechanism;
import first.robot.util.FieldGeometry;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.function.Supplier;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.interpolation.TimeInterpolatableBuffer;
import org.wpilib.math.util.Units;
import org.wpilib.networktables.DoubleArrayPublisher;
import org.wpilib.networktables.DoubleArraySubscriber;
import org.wpilib.networktables.DoublePublisher;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.system.Timer;

/**
 * A pretend Limelight. It looks at where the simulated robot really is, works out which AprilTags
 * the camera could see, and publishes a MegaTag2 result to the same NetworkTables topics a real
 * Limelight would. {@link VisionIOLimelight} reads it without knowing the difference.
 *
 * <p>Realism it adds on purpose: frames arrive at a fixed rate, each one describes where the robot
 * was when the image was taken (latency), and the position has noise that grows with distance and
 * shrinks with more tags. Not modeled: motion blur, occlusion, lighting, or wrong tag detections.
 */
public class LimelightSim implements SimulatedMechanism {
  // Match the real pipeline's frame rate.
  private static final double FRAME_PERIOD_SECS = 1.0 / 90.0;
  private static final double LATENCY_SECS = 0.025;
  // Limelight 4 field of view, and the farthest a tag is still decoded reliably.
  private static final double HORIZONTAL_FOV_RAD = Units.degreesToRadians(82.0);
  private static final double VERTICAL_FOV_RAD = Units.degreesToRadians(56.2);
  private static final double MAX_RANGE_METERS = 6.0;
  // Tags seen too edge-on don't decode. 0 would be face-on.
  private static final double MAX_VIEW_ANGLE_RAD = Units.degreesToRadians(70.0);
  // Already counted in Battery.BASE_LOAD_AMPS, so the camera adds nothing on top.
  private static final double CAMERA_AMPS = 0.0;

  // Where the robot code says this camera is mounted (see VisionIOLimelight). A real Limelight
  // that hasn't been told uses the pose from its web UI; this one assumes robot center until then.
  private final DoubleArraySubscriber cameraPoseSubscriber;
  private final Supplier<Pose2d> truePoseSupplier;
  private final TimeInterpolatableBuffer<Pose2d> truePoseHistory =
      TimeInterpolatableBuffer.createBuffer(1.0);
  private final Random random = new Random();

  private final DoubleArrayPublisher botposePublisher;
  private final DoublePublisher latencyPublisher;
  private final DoublePublisher txPublisher;
  private final DoublePublisher tyPublisher;
  private final DoubleArraySubscriber orientationSubscriber;

  private double secsSinceFrame = 0.0;

  /**
   * @param name the camera's NT table, matching the name its VisionIOLimelight reads
   * @param truePoseSupplier where the simulated robot really is
   */
  public LimelightSim(String name, Supplier<Pose2d> truePoseSupplier) {
    this.truePoseSupplier = truePoseSupplier;
    var table = NetworkTableInstance.getDefault().getTable(name);
    cameraPoseSubscriber =
        table.getDoubleArrayTopic("camerapose_robotspace_set").subscribe(new double[] {});
    botposePublisher = table.getDoubleArrayTopic("botpose_orb_wpiblue").publish();
    latencyPublisher = table.getDoubleTopic("tl").publish();
    txPublisher = table.getDoubleTopic("tx").publish();
    tyPublisher = table.getDoubleTopic("ty").publish();
    orientationSubscriber =
        table.getDoubleArrayTopic("robot_orientation_set").subscribe(new double[] {});
  }

  @Override
  public double update(double dtSeconds, double batteryVolts) {
    // Not Timer.getTimestamp(): AdvantageKit freezes that once per robot loop for replay, and this
    // runs on the SimWorld thread between loops.
    double now = Timer.getMonotonicTimestamp();
    truePoseHistory.addSample(now, truePoseSupplier.get());

    secsSinceFrame += dtSeconds;
    if (secsSinceFrame < FRAME_PERIOD_SECS) {
      return CAMERA_AMPS;
    }
    secsSinceFrame = 0.0;

    // The frame shows the robot as it was when the image was taken, not as it is now.
    var robotAtCapture = truePoseHistory.getSample(now - LATENCY_SECS);
    if (robotAtCapture.isEmpty()) {
      return CAMERA_AMPS;
    }
    double[] pushedPose = cameraPoseSubscriber.get();
    var robotToCamera =
        pushedPose.length == 6
            ? VisionIOLimelight.fromLimelightCameraPose(pushedPose)
            : Transform3d.ZERO;
    var cameraPose = new Pose3d(robotAtCapture.get()).transformBy(robotToCamera);
    var visible = visibleTags(cameraPose);

    // Small jitter keeps the value changing, which is how VisionIOLimelight knows we're alive.
    double latencyMs = LATENCY_SECS * 1000.0 + random.nextDouble() * 0.1;
    latencyPublisher.set(latencyMs);
    botposePublisher.set(buildBotpose(robotAtCapture.get(), visible, latencyMs));

    if (visible.isEmpty()) {
      txPublisher.set(0.0);
      tyPublisher.set(0.0);
    } else {
      // Aim data for the closest tag. Limelight reports tx positive to the right.
      var closest = visible.get(0);
      var inCamera = closest.tagInCamera;
      txPublisher.set(-Units.radiansToDegrees(Math.atan2(inCamera.getY(), inCamera.getX())));
      tyPublisher.set(Units.radiansToDegrees(Math.atan2(inCamera.getZ(), inCamera.getX())));
    }
    return CAMERA_AMPS;
  }

  private record VisibleTag(int id, Pose3d tagInCamera, double distance) {}

  /** Tags in front of the camera, inside its field of view and range, and facing it. */
  private List<VisibleTag> visibleTags(Pose3d cameraPose) {
    List<VisibleTag> visible = new ArrayList<>();
    for (var tag : FieldGeometry.FIELD.getTags()) {
      // Tag position as the camera sees it: +x ahead, +y left, +z up.
      var tagInCamera = tag.getPose().relativeTo(cameraPose);
      double x = tagInCamera.getX();
      double distance = tagInCamera.getTranslation().getNorm();
      if (x <= 0.0 || distance > MAX_RANGE_METERS) {
        continue;
      }
      if (Math.abs(Math.atan2(tagInCamera.getY(), x)) > HORIZONTAL_FOV_RAD / 2
          || Math.abs(Math.atan2(tagInCamera.getZ(), x)) > VERTICAL_FOV_RAD / 2) {
        continue;
      }
      // A tag's pose points out of its printed face, so the camera must be on the +x side of it,
      // and not too far off to the side.
      var cameraInTag = cameraPose.relativeTo(tag.getPose()).getTranslation();
      double viewAngle =
          Math.atan2(Math.hypot(cameraInTag.getY(), cameraInTag.getZ()), cameraInTag.getX());
      if (cameraInTag.getX() <= 0.0 || viewAngle > MAX_VIEW_ANGLE_RAD) {
        continue;
      }
      visible.add(new VisibleTag(tag.getID(), tagInCamera, distance));
    }
    visible.sort((a, b) -> Double.compare(a.distance, b.distance));
    return visible;
  }

  /** Builds a botpose_orb_wpiblue array, the same layout VisionIOLimelight parses. */
  private double[] buildBotpose(Pose2d robot, List<VisibleTag> visible, double latencyMs) {
    double[] values = new double[11 + 7 * visible.size()];
    values[6] = latencyMs;
    values[7] = visible.size();
    if (visible.isEmpty()) {
      return values;
    }

    double averageDistance = visible.stream().mapToDouble(VisibleTag::distance).average().orElse(0);
    // Same model Vision uses to decide how much to trust a result, so the trust is accurate here.
    double stdDev =
        VisionConstants.LINEAR_STD_DEV_BASELINE
            * averageDistance
            * averageDistance
            / visible.size();

    // MegaTag2 takes the heading from the robot instead of the image, so report whatever heading
    // the robot sent. If it's wrong, the estimate is wrong too, as on a real robot.
    double[] orientation = orientationSubscriber.get();
    double headingDeg = orientation.length > 0 ? orientation[0] : robot.getRotation().getDegrees();

    values[0] = robot.getX() + random.nextGaussian() * stdDev;
    values[1] = robot.getY() + random.nextGaussian() * stdDev;
    values[5] = headingDeg;
    values[9] = averageDistance;
    for (int i = 0; i < visible.size(); i++) {
      // Only the ID and distance are filled in; VisionIOLimelight doesn't read the rest.
      values[11 + 7 * i] = visible.get(i).id;
      values[11 + 7 * i + 4] = visible.get(i).distance;
    }
    return values;
  }

  /** Forgets the pose history, for when the simulated robot is teleported. */
  public void clearHistory() {
    truePoseHistory.clear();
  }
}
