package first.robot.subsystems.vision;

import first.robot.field.FieldGeometry;
import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;
import java.util.Map;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N3;
import org.wpilib.util.Alert;
import org.wpilib.util.Alert.Level;

/**
 * Turns camera pose estimates into corrections for the drive's pose estimator.
 *
 * <p>Each loop it reads every camera, throws out estimates that can't be right, works out how much
 * to trust the rest, and hands them to the estimator through {@link VisionConsumer}. It has no
 * motors, so it isn't a command Mechanism; the robot just calls {@link #periodic()} every loop,
 * after the drive has updated odometry.
 */
public class Vision {
  /** Matches Drive.addVisionMeasurement, so that method can be passed in directly. */
  @FunctionalInterface
  public interface VisionConsumer {
    void accept(Pose2d robotPose, double timestampSeconds, Matrix<N3, N1> stdDevs);
  }

  /** Why an observation was thrown out. Each reason gets its own running count in the log. */
  enum RejectReason {
    NO_TAGS,
    OFF_FIELD,
    TOO_FAR,
    SPINNING,
    // MegaTag1 only: one tag can't give a trustworthy heading.
    ONE_TAG
  }

  private final VisionConsumer consumer;
  private final DoubleSupplier yawVelocityRadPerSec;
  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;
  private final Map<RejectReason, Integer> rejectCounts = new EnumMap<>(RejectReason.class);

  /**
   * @param yawVelocityRadPerSec how fast the robot is turning. Estimates are thrown out while it
   *     spins fast (see rejectReason)
   */
  public Vision(VisionConsumer consumer, DoubleSupplier yawVelocityRadPerSec, VisionIO... io) {
    this.consumer = consumer;
    this.yawVelocityRadPerSec = yawVelocityRadPerSec;
    this.io = io;
    inputs = new VisionIOInputsAutoLogged[io.length];
    disconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < io.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
      disconnectedAlerts[i] =
          new Alert(
              "Vision",
              "Camera" + i + "Disconnected",
              "Vision camera " + i + " is disconnected.",
              Level.MEDIUM);
    }
    for (var reason : RejectReason.values()) {
      rejectCounts.put(reason, 0);
    }
  }

  /** Horizontal angle to the best target on one camera, for simple aiming. */
  public Rotation2d getTargetX(int cameraIndex) {
    return inputs[cameraIndex].latestTargetObservation.tx();
  }

  public void periodic() {
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/Camera" + i, inputs[i]);
    }

    List<Pose3d> allAccepted = new ArrayList<>();
    List<Pose3d> allRejected = new ArrayList<>();
    for (int camera = 0; camera < io.length; camera++) {
      disconnectedAlerts[camera].set(!inputs[camera].connected);

      List<Pose3d> tagPoses = new ArrayList<>();
      for (int tagId : inputs[camera].tagIds) {
        FieldGeometry.FIELD.getTagPose(tagId).ifPresent(tagPoses::add);
      }

      List<Pose3d> accepted = new ArrayList<>();
      List<Pose3d> rejected = new ArrayList<>();
      for (var observation : inputs[camera].poseObservations) {
        var pose = observation.pose();
        var reason = rejectReason(observation, yawVelocityRadPerSec.getAsDouble());
        if (reason != null) {
          rejectCounts.merge(reason, 1, Integer::sum);
          rejected.add(pose);
          continue;
        }
        accepted.add(pose);

        // A standard deviation is roughly how far off this estimate might be. The bigger it is,
        // the less the pose estimator moves toward it. Error grows with distance squared and
        // shrinks with more tags in view. Dividing by the tag count is a rule of thumb, not
        // derived: averaging independent errors would only divide by its square root, but tags
        // spread across the image also pin down the angle better.
        double distance = observation.averageTagDistance();
        double scale =
            distance
                * distance
                / observation.tagCount()
                * VisionConstants.CAMERA_STD_DEV_FACTORS[camera];
        // Each solver gets trusted for what it's good at. An infinite standard deviation means
        // "ignore this part".
        var stdDevs =
            switch (observation.type()) {
              // Heading only. MegaTag2's position is much steadier, so MegaTag1's is ignored.
              case MEGATAG_1 ->
                  VecBuilder.fill(
                      Double.POSITIVE_INFINITY,
                      Double.POSITIVE_INFINITY,
                      VisionConstants.ANGULAR_STD_DEV_BASELINE * scale);
              // Position only: its heading came from the robot, so it can't correct it. Spinning
              // adds the heading-mismatch error on top (see
              // VisionConstants.MEGATAG2_HEADING_DELAY_SECS).
              case MEGATAG_2 -> {
                double spinError =
                    distance
                        * Math.abs(yawVelocityRadPerSec.getAsDouble())
                        * VisionConstants.MEGATAG2_HEADING_DELAY_SECS
                        * VisionConstants.CAMERA_STD_DEV_FACTORS[camera];
                double linear = VisionConstants.LINEAR_STD_DEV_BASELINE * scale + spinError;
                yield VecBuilder.fill(linear, linear, Double.POSITIVE_INFINITY);
              }
            };
        consumer.accept(pose.toPose2d(), observation.timestamp(), stdDevs);
      }

      String prefix = "Vision/Camera" + camera;
      Logger.recordOutput(prefix + "/TagPoses", tagPoses.toArray(new Pose3d[0]));
      Logger.recordOutput(prefix + "/RobotPosesAccepted", accepted.toArray(new Pose3d[0]));
      Logger.recordOutput(prefix + "/RobotPosesRejected", rejected.toArray(new Pose3d[0]));
      allAccepted.addAll(accepted);
      allRejected.addAll(rejected);
    }
    Logger.recordOutput("Vision/Summary/RobotPosesAccepted", allAccepted.toArray(new Pose3d[0]));
    Logger.recordOutput("Vision/Summary/RobotPosesRejected", allRejected.toArray(new Pose3d[0]));
    // Running totals. A count that climbs during a match points at the filter to look at.
    rejectCounts.forEach(
        (reason, count) -> Logger.recordOutput("Vision/Summary/Rejected/" + reason, count));
  }

  /**
   * Why an observation can't be trusted, or null if it can. Package-private for the unit tests.
   *
   * <p>The first two catch estimates that are plainly wrong. The rest catch estimates that might
   * look fine but are likely off: far tags are a few pixels wide, MegaTag2 solves using the heading
   * the robot sent it, which doesn't match the moment of the picture while the robot spins quickly,
   * and MegaTag1's heading from a single tag can flip.
   */
  static RejectReason rejectReason(
      VisionIO.PoseObservation observation, double yawVelocityRadPerSec) {
    var pose = observation.pose();
    if (observation.tagCount() == 0) {
      return RejectReason.NO_TAGS;
    }
    // A robot can't be floating, underground, or outside the field.
    if (Math.abs(pose.getZ()) > VisionConstants.MAX_Z_ERROR_METERS
        || pose.getX() < 0.0
        || pose.getX() > FieldGeometry.LENGTH_METERS
        || pose.getY() < 0.0
        || pose.getY() > FieldGeometry.WIDTH_METERS) {
      return RejectReason.OFF_FIELD;
    }
    if (observation.averageTagDistance() > VisionConstants.MAX_TAG_DISTANCE_METERS) {
      return RejectReason.TOO_FAR;
    }
    if (Math.abs(yawVelocityRadPerSec) > VisionConstants.MAX_YAW_VELOCITY_RAD_PER_SEC) {
      return RejectReason.SPINNING;
    }
    if (observation.type() == VisionIO.ObservationType.MEGATAG_1
        && observation.tagCount() < VisionConstants.MEGATAG_1_MIN_TAGS) {
      return RejectReason.ONE_TAG;
    }
    return null;
  }
}
