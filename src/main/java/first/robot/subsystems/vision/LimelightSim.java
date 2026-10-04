package first.robot.subsystems.vision;

import first.robot.field.FieldGeometry;
import first.robot.sim.SimulatedMechanism;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.function.Supplier;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.interpolation.TimeInterpolatableBuffer;
import org.wpilib.math.util.Units;
import org.wpilib.networktables.DoubleArrayPublisher;
import org.wpilib.networktables.DoubleArraySubscriber;
import org.wpilib.networktables.DoublePublisher;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.system.Timer;

/**
 * A pretend Limelight. It looks at where the simulated robot really is, works out which AprilTags
 * the camera could see, and publishes MegaTag1 and MegaTag2 results to the same NetworkTables
 * topics a real Limelight would. {@link VisionIOLimelight} reads them without knowing the
 * difference.
 *
 * <p>Realism it adds on purpose: frames arrive at a fixed rate, each one describes where the robot
 * was when the image was taken (latency), and the position has noise that grows with distance and
 * shrinks with more tags. Not modeled: motion blur, occlusion, lighting, wrong tag detections, or
 * MegaTag1's heading flipping when it sees only one tag.
 */
public class LimelightSim implements SimulatedMechanism {
  // FUDGE: real-world effects this sim leaves out. Both set to "no effect" here.

  // FUDGE: how noisy the simulated camera is, compared with how much Vision trusts it
  // (VisionConstants.LINEAR_STD_DEV_BASELINE). At 1.0 the trust is exactly right. A real camera
  // usually isn't that tidy: try 2.0 to see what trusting a camera too much does to the pose.
  private static final double FUDGE_NOISE_SCALE = 1.0;

  // FUDGE: the chance that a frame is badly wrong, as when a real camera misreads a tag or catches
  // a reflection. Such a frame lands about a meter from the truth. Try 0.02 to see how much the
  // filters in Vision let through.
  private static final double FUDGE_OUTLIER_CHANCE = 0.0;

  private static final double FUDGE_OUTLIER_METERS = 1.0;

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

  private final DoubleArrayPublisher megatag1Publisher;
  private final DoubleArrayPublisher megatag2Publisher;
  private final DoublePublisher latencyPublisher;
  private final DoublePublisher txPublisher;
  private final DoublePublisher tyPublisher;
  private final DoubleArraySubscriber orientationSubscriber;

  private double secsSinceFrame = 0.0;
  private Pose2d lastTruePose = null;

  /**
   * @param name the camera's NT table, matching the name its VisionIOLimelight reads
   * @param truePoseSupplier where the simulated robot really is
   */
  public LimelightSim(String name, Supplier<Pose2d> truePoseSupplier) {
    this.truePoseSupplier = truePoseSupplier;
    var table = NetworkTableInstance.getDefault().getTable(name);
    cameraPoseSubscriber =
        table.getDoubleArrayTopic("camerapose_robotspace_set").subscribe(new double[] {});
    megatag1Publisher = table.getDoubleArrayTopic("botpose_wpiblue").publish();
    megatag2Publisher = table.getDoubleArrayTopic("botpose_orb_wpiblue").publish();
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
    var truePose = truePoseSupplier.get();
    // A jump no robot could drive in one step means the simulated robot was teleported (a pose
    // reset). Forget the old spot, or the next few frames would come from in between the two.
    // Done here, on the SimWorld thread, because the history isn't safe to touch from two threads.
    if (lastTruePose != null && isTeleport(lastTruePose, truePose)) {
      truePoseHistory.clear();
    }
    lastTruePose = truePose;
    truePoseHistory.addSample(now, truePose);

    secsSinceFrame += dtSeconds;
    if (secsSinceFrame < FRAME_PERIOD_SECS) {
      return CAMERA_AMPS;
    }
    // Subtract rather than reset to zero: the sim steps every 4 ms, which doesn't divide evenly
    // into a frame period, and resetting would round every frame up to the next step (83 fps
    // instead of 90).
    secsSinceFrame -= FRAME_PERIOD_SECS;

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
    megatag1Publisher.set(buildMegatag1(robotAtCapture.get(), visible, latencyMs));
    megatag2Publisher.set(buildMegatag2(robotAtCapture.get(), visible, latencyMs));

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

  /**
   * The botpose array layout both solvers share, with no pose filled in yet: latency, tag count,
   * average distance, and each tag's ID and distance. Empty of tags when none are visible.
   */
  private static double[] emptyBotpose(List<VisibleTag> visible, double latencyMs) {
    double[] values = new double[11 + 7 * visible.size()];
    values[6] = latencyMs;
    values[7] = visible.size();
    values[9] = visible.stream().mapToDouble(VisibleTag::distance).average().orElse(0);
    for (int i = 0; i < visible.size(); i++) {
      // Only the ID and distance are filled in; VisionIOLimelight doesn't read the rest.
      values[11 + 7 * i] = visible.get(i).id;
      values[11 + 7 * i + 4] = visible.get(i).distance;
    }
    return values;
  }

  /** How much to scale the trust baselines for this view, the same way Vision does. */
  private static double noiseScale(List<VisibleTag> visible) {
    double averageDistance = visible.stream().mapToDouble(VisibleTag::distance).average().orElse(0);
    return averageDistance * averageDistance / visible.size() * FUDGE_NOISE_SCALE;
  }

  /**
   * A botpose_wpiblue array: MegaTag1, which solves position and heading from the image alone. Its
   * heading gets the noise Vision expects (VisionConstants.ANGULAR_STD_DEV_BASELINE). Its position
   * is noisier than MegaTag2's; Vision ignores it, so the exact amount doesn't matter.
   */
  private double[] buildMegatag1(Pose2d robot, List<VisibleTag> visible, double latencyMs) {
    double[] values = emptyBotpose(visible, latencyMs);
    if (visible.isEmpty()) {
      return values;
    }
    double scale = noiseScale(visible);
    double linearStdDev = 3 * VisionConstants.LINEAR_STD_DEV_BASELINE * scale;
    values[0] = robot.getX() + random.nextGaussian() * linearStdDev;
    values[1] = robot.getY() + random.nextGaussian() * linearStdDev;
    values[5] =
        robot.getRotation().getDegrees()
            + Units.radiansToDegrees(
                random.nextGaussian() * VisionConstants.ANGULAR_STD_DEV_BASELINE * scale);
    addOutlier(values);
    return values;
  }

  /** A botpose_orb_wpiblue array: MegaTag2, which solves position using the robot's heading. */
  private double[] buildMegatag2(Pose2d robot, List<VisibleTag> visible, double latencyMs) {
    double[] values = emptyBotpose(visible, latencyMs);
    if (visible.isEmpty()) {
      return values;
    }

    // MegaTag2 takes the heading from the robot instead of the image. The camera sees which way
    // the tags are from the robot, and turns that into field directions using the heading it was
    // sent. A heading off by some angle swings the answer around the tags by that same angle, so
    // the position comes out wrong by roughly distance * angle (in radians): 2 degrees off at 4 m
    // is about 14 cm. Reproduce that here, as on a real robot.
    double[] orientation = orientationSubscriber.get();
    var sentHeading =
        orientation.length > 0 ? Rotation2d.fromDegrees(orientation[0]) : robot.getRotation();
    var headingError = sentHeading.minus(robot.getRotation());
    var tagCenter = Translation2d.ZERO;
    for (var tag : visible) {
      var tagPose = FieldGeometry.FIELD.getTagPose(tag.id).orElseThrow();
      tagCenter = tagCenter.plus(tagPose.toPose2d().getTranslation().div(visible.size()));
    }
    var solved = tagCenter.plus(robot.getTranslation().minus(tagCenter).rotateBy(headingError));

    // The same noise Vision expects (unless FUDGE_NOISE_SCALE says otherwise), so the trust is
    // accurate here.
    double stdDev = VisionConstants.LINEAR_STD_DEV_BASELINE * noiseScale(visible);
    values[0] = solved.getX() + random.nextGaussian() * stdDev;
    values[1] = solved.getY() + random.nextGaussian() * stdDev;
    values[5] = sentHeading.getDegrees();
    addOutlier(values);
    return values;
  }

  /** Once in a while (FUDGE_OUTLIER_CHANCE), throws the position about a meter off. */
  private void addOutlier(double[] values) {
    if (random.nextDouble() < FUDGE_OUTLIER_CHANCE) {
      double direction = random.nextDouble() * 2 * Math.PI;
      values[0] += FUDGE_OUTLIER_METERS * Math.cos(direction);
      values[1] += FUDGE_OUTLIER_METERS * Math.sin(direction);
    }
  }

  /** Farther or more turned than a robot can manage in one sim step (a few millimeters). */
  private static boolean isTeleport(Pose2d before, Pose2d after) {
    return before.getTranslation().getDistance(after.getTranslation()) > 0.25
        || Math.abs(after.getRotation().minus(before.getRotation()).getRadians()) > 0.5;
  }
}
