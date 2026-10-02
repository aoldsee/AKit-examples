package first.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import first.robot.field.FieldGeometry;
import first.robot.subsystems.vision.VisionIO.PoseObservation;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N3;

/**
 * Checks Vision's filtering and trust rules by feeding it made-up camera results through a fake IO,
 * then looking at what it passes on to the pose estimator. No camera, field, or robot needed.
 */
class VisionTest {
  /** Hands Vision whatever observations the test sets, once each. */
  private static class FakeVisionIO implements VisionIO {
    PoseObservation[] next = new PoseObservation[0];

    @Override
    public void updateInputs(VisionIOInputs inputs) {
      inputs.connected = true;
      inputs.poseObservations = next;
      next = new PoseObservation[0];
    }
  }

  private record Measurement(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs) {}

  private static final FakeVisionIO camera = new FakeVisionIO();
  // What the "gyro" reports. Each test starts with the robot not turning.
  private static double yawVelocity = 0.0;
  private static final List<Measurement> sent = new ArrayList<>();
  // One Vision for the whole class: its alerts are registered by name and can't be created twice.
  private static Vision vision;

  private static final double MID_X = FieldGeometry.FIELD.getFieldLength() / 2;
  private static final double MID_Y = FieldGeometry.FIELD.getFieldWidth() / 2;

  @BeforeAll
  static void setup() {
    vision =
        new Vision(
            (pose, timestamp, stdDevs) -> sent.add(new Measurement(pose, timestamp, stdDevs)),
            () -> yawVelocity,
            camera);
  }

  @BeforeEach
  void clear() {
    sent.clear();
    yawVelocity = 0.0;
  }

  private static PoseObservation observation(
      double x, double y, double z, int tagCount, double distance) {
    return new PoseObservation(1.0, new Pose3d(x, y, z, Rotation3d.ZERO), tagCount, distance);
  }

  private static void feed(PoseObservation... observations) {
    camera.next = observations;
    vision.periodic();
  }

  @Test
  void goodObservationIsPassedOn() {
    feed(observation(MID_X, MID_Y, 0.0, 1, 2.0));

    assertEquals(1, sent.size());
    var measurement = sent.get(0);
    assertEquals(MID_X, measurement.pose().getX(), 1e-9);
    assertEquals(1.0, measurement.timestamp(), 1e-9);
    // One tag at 2 m: baseline * distance^2 / tags.
    assertEquals(
        VisionConstants.LINEAR_STD_DEV_BASELINE * 4 / 1, measurement.stdDevs().get(0, 0), 1e-9);
    // MegaTag2 can't correct heading, so heading is never trusted.
    assertEquals(Double.POSITIVE_INFINITY, measurement.stdDevs().get(2, 0));
  }

  @Test
  void closerAndMoreTagsMeansMoreTrust() {
    feed(observation(MID_X, MID_Y, 0.0, 1, 3.0), observation(MID_X, MID_Y, 0.0, 2, 1.0));

    assertEquals(2, sent.size());
    double farSingleTag = sent.get(0).stdDevs().get(0, 0);
    double closeTwoTags = sent.get(1).stdDevs().get(0, 0);
    // A smaller standard deviation means the estimator trusts it more.
    assertTrue(closeTwoTags < farSingleTag);
    assertEquals(VisionConstants.LINEAR_STD_DEV_BASELINE * 9 / 1, farSingleTag, 1e-9);
    assertEquals(VisionConstants.LINEAR_STD_DEV_BASELINE * 1 / 2, closeTwoTags, 1e-9);
  }

  @Test
  void impossibleObservationsAreDropped() {
    feed(
        observation(MID_X, MID_Y, 0.0, 0, 2.0), // no tags behind it
        observation(MID_X, MID_Y, 1.5, 1, 2.0), // robot floating in the air
        observation(-0.5, MID_Y, 0.0, 1, 2.0), // behind the alliance wall
        observation(MID_X, FieldGeometry.FIELD.getFieldWidth() + 0.5, 0.0, 1, 2.0)); // off the side

    assertEquals(0, sent.size());
  }

  @Test
  void farTagsAreDropped() {
    feed(observation(MID_X, MID_Y, 0.0, 1, VisionConstants.MAX_TAG_DISTANCE_METERS + 0.5));

    assertEquals(0, sent.size());
    assertEquals(
        Vision.RejectReason.TOO_FAR,
        Vision.rejectReason(
            observation(MID_X, MID_Y, 0.0, 1, VisionConstants.MAX_TAG_DISTANCE_METERS + 0.5), 0.0));
  }

  @Test
  void spinningMeansLessTrust() {
    // One tag at 2 m, turning one radian per second.
    yawVelocity = 1.0;
    feed(observation(MID_X, MID_Y, 0.0, 1, 2.0));

    // The usual baseline * distance^2 / tags, plus distance * spin rate * heading delay.
    assertEquals(
        VisionConstants.LINEAR_STD_DEV_BASELINE * 4
            + 2.0 * 1.0 * VisionConstants.MEGATAG2_HEADING_DELAY_SECS,
        sent.get(0).stdDevs().get(0, 0),
        1e-9);
  }

  @Test
  void nothingIsTrustedWhileSpinningFast() {
    var good = observation(MID_X, MID_Y, 0.0, 2, 1.0);

    // Turning just under the limit, either way, is fine.
    yawVelocity = -0.9 * VisionConstants.MAX_YAW_VELOCITY_RAD_PER_SEC;
    feed(good);
    assertEquals(1, sent.size());

    // Even a close, two-tag estimate is dropped while spinning faster than that.
    yawVelocity = 1.1 * VisionConstants.MAX_YAW_VELOCITY_RAD_PER_SEC;
    feed(good);
    assertEquals(1, sent.size());
    assertEquals(
        Vision.RejectReason.SPINNING,
        Vision.rejectReason(good, -1.1 * VisionConstants.MAX_YAW_VELOCITY_RAD_PER_SEC));
  }
}
