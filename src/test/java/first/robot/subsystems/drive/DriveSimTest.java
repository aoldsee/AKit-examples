package first.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.ctre.phoenix6.SignalLogger;
import first.robot.field.AlignTargets;
import first.robot.field.FieldGeometry;
import first.robot.sim.SimWorld;
import first.robot.subsystems.vision.LimelightSim;
import first.robot.subsystems.vision.Vision;
import first.robot.subsystems.vision.VisionConstants;
import first.robot.subsystems.vision.VisionIO;
import first.robot.subsystems.vision.VisionIOLimelight;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.hardware.hal.RobotMode;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.simulation.DriverStationSim;

/**
 * Runs the sim IO (real TalonFX/Pigeon code underneath) against SwerveDriveSim in real time.
 * Phoenix sim devices run their firmware on wall-clock time, so this can't be stepped faster than
 * real time.
 */
@Tag("sim")
class DriveSimTest {
  private static SimWorld simWorld;
  private static SwerveDriveSim driveSim;
  private static Vision vision;
  private static final List<LimelightSim> limelightSims = new ArrayList<>();
  private static Drive drive;

  @BeforeAll
  static void setup() {
    HAL.initialize();
    SignalLogger.enableAutoLogging(false);
    simWorld = new SimWorld();
    driveSim = new SwerveDriveSim();
    simWorld.add("Drive", driveSim);
    drive =
        new Drive(
            new GyroIOPigeon2Sim(driveSim),
            new ModuleIOTalonFXSim(DriveConstants.MODULES[0], driveSim),
            new ModuleIOTalonFXSim(DriveConstants.MODULES[1], driveSim),
            new ModuleIOTalonFXSim(DriveConstants.MODULES[2], driveSim),
            new ModuleIOTalonFXSim(DriveConstants.MODULES[3], driveSim));
    // Vision is built here but only stepped in the vision test, so it can't mask odometry errors
    // the other tests are looking for.
    var visionIOs = new VisionIO[VisionConstants.CAMERA_NAMES.length];
    for (int i = 0; i < visionIOs.length; i++) {
      var limelightSim = new LimelightSim(VisionConstants.CAMERA_NAMES[i], driveSim::getTruePose);
      simWorld.add("Camera" + i, limelightSim);
      limelightSims.add(limelightSim);
      visionIOs[i] =
          new VisionIOLimelight(
              VisionConstants.CAMERA_NAMES[i],
              VisionConstants.ROBOT_TO_CAMERA[i],
              drive::getRotation);
    }
    vision =
        new Vision(
            drive::addVisionMeasurement, () -> drive.getChassisVelocities().omega, visionIOs);
    simWorld.start();
    DriverStationSim.setRobotMode(RobotMode.TELEOPERATED);
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
  }

  /** Tests share one simulated robot, so each starts stopped at the odometry origin. */
  @BeforeEach
  void settleAndZero() throws InterruptedException {
    run(new ChassisVelocities(), 0.5);
    drive.setPose(Pose2d.ZERO);
  }

  @AfterAll
  static void teardown() {
    simWorld.close();
  }

  private static void run(ChassisVelocities velocities, double seconds)
      throws InterruptedException {
    long end = System.nanoTime() + (long) (seconds * 1e9);
    while (System.nanoTime() < end) {
      DriverStationSim.notifyNewData();
      drive.periodic();
      drive.runVelocity(velocities);
      Thread.sleep(20);
    }
  }

  /** Runs a command through the real scheduler for a while, then cancels it. */
  private static void runCommand(Command command, double seconds) throws InterruptedException {
    var scheduler = Scheduler.getDefault();
    scheduler.schedule(command);
    long end = System.nanoTime() + (long) (seconds * 1e9);
    while (System.nanoTime() < end) {
      DriverStationSim.notifyNewData();
      drive.periodic();
      scheduler.run();
      Thread.sleep(20);
    }
    scheduler.cancel(command);
    scheduler.run();
  }

  @Test
  void encoderOffsetsUndoAWheelAngle() throws InterruptedException {
    // Point every module 30 degrees left, as if it had been bumped off straight, then disable.
    var angle = Rotation2d.fromDegrees(30.0);
    long end = System.nanoTime() + 1_000_000_000L;
    while (System.nanoTime() < end) {
      DriverStationSim.notifyNewData();
      drive.periodic();
      drive.runDirect(angle, 0.0);
      Thread.sleep(20);
    }
    DriverStationSim.setEnabled(false);
    try {
      // Only read inputs while disabling. The Talons keep obeying commands for a moment after the
      // disable, and a stop request would turn the wheels back to the last kinematic heading (0).
      end = System.nanoTime() + 200_000_000L;
      while (System.nanoTime() < end) {
        DriverStationSim.notifyNewData();
        drive.periodic();
        Thread.sleep(20);
      }
      runCommand(DriveCharacterization.findEncoderOffsets(drive), 0.2);
      double[] offsets = DriveCharacterization.encoderOffsets(drive.getModuleAbsoluteAngles());
      System.out.println("SIMTEST encoder offsets: " + java.util.Arrays.toString(offsets));
      for (double offset : offsets) {
        // Subtracting 30 degrees (1/12 of a turn) brings the module back to reading 0.
        assertEquals(-1.0 / 12.0, offset, 0.005);
      }
    } finally {
      DriverStationSim.setEnabled(true);
      DriverStationSim.notifyNewData();
    }
  }

  @Test
  void systemsCheckPassesOnAHealthyDrivetrain() throws InterruptedException {
    List<String> failures = new ArrayList<>();
    runCommand(DriveSystemsCheck.create(drive, failures::addAll), 3.5);
    System.out.println("SIMTEST systems check failures: " + failures);
    assertEquals(List.of(), failures);
  }

  @Test
  void feedforwardCharacterizationRecoversSimMotor() throws InterruptedException {
    run(new ChassisVelocities(), 0.5);
    var result = new AtomicReference<DriveCharacterization.FeedforwardFit>();
    // 2 s settle, then 10 s of ramp up to 1 V.
    runCommand(DriveCharacterization.feedforwardCharacterization(drive, result::set), 12.0);

    var fit = result.get();
    System.out.println("SIMTEST FF fit: " + fit);
    // The sim is built from DRIVE_KS and DRIVE_KV, so characterizing it should measure them back.
    // That isn't circular: the routine only sends voltages and watches the wheels, and never reads
    // those constants, so it's measuring them the same way it would on a real robot.
    double expectedKV = DriveConstants.DRIVE_KV;
    assertEquals(expectedKV, fit.kV(), expectedKV * 0.1, "kV");
    assertEquals(DriveConstants.DRIVE_KS, fit.kS(), 0.1, "kS");
  }

  @Test
  void wheelRadiusCharacterizationRecoversWheelRadius() throws InterruptedException {
    run(new ChassisVelocities(), 0.5);
    var result = new AtomicReference<DriveCharacterization.WheelRadiusResult>();
    runCommand(DriveCharacterization.wheelRadiusCharacterization(drive, result::set), 12.0);

    var measured = result.get();
    System.out.println("SIMTEST wheel radius: " + measured);
    double expected = DriveConstants.WHEEL_RADIUS_METERS;
    assertEquals(expected, measured.radiusMeters(), expected * 0.05, "wheel radius");
  }

  @Test
  void sysIdDynamicDrivesInRequestedDirection() throws InterruptedException {
    // 1 s settle plus 1 s of the 7 V step. Steady state at 7 V is about 2.9 m/s, so even with the
    // ramp up this covers well over a meter.
    runCommand(
        DriveCharacterization.sysIdDynamic(drive, DriveCharacterization.SysIdDirection.REVERSE),
        2.0);
    var pose = drive.getPose();
    System.out.println("SIMTEST sysid dynamic reverse pose: " + pose);
    assertTrue(pose.getX() < -1.0, "moved backward");
    assertEquals(0.0, pose.getY(), 0.1, "no sideways drift");
  }

  @Test
  void visionCorrectsWrongOdometry() throws InterruptedException {
    // Stand 2.5 m in front of the first tag that leaves room on the field, facing it, so the
    // front camera has a clear view.
    Pose2d truth = null;
    for (var tag : FieldGeometry.FIELD.getTags()) {
      var candidate =
          tag.getPose().toPose2d().transformBy(new Transform2d(2.5, 0.0, Rotation2d.k180deg));
      if (candidate.getX() > 1.0
          && candidate.getX() < FieldGeometry.FIELD.getFieldLength() - 1.0
          && candidate.getY() > 1.0
          && candidate.getY() < FieldGeometry.FIELD.getFieldWidth() - 1.0
          && tag.getPose().getZ() < 1.5) {
        truth = candidate;
        break;
      }
    }
    assertNotNull(truth, "no tag with room in front of it");

    driveSim.resetTruePose(truth);
    limelightSims.forEach(LimelightSim::clearHistory);
    // Odometry starts a meter off; only vision can fix that.
    drive.setPose(
        new Pose2d(truth.getTranslation().plus(new Translation2d(0.8, -0.6)), truth.getRotation()));

    // Each camera frame is a few centimeters off (that's the simulated noise), and the estimate
    // follows the frames closely, so it jitters too. So the test judges it the way a person would
    // by
    // eye: the average error over the last half second.
    double errorSum = 0.0;
    int errorSamples = 0;
    long start = System.nanoTime();
    while (System.nanoTime() - start < 2_500_000_000L) {
      DriverStationSim.notifyNewData();
      drive.periodic();
      vision.periodic();
      drive.runVelocity(new ChassisVelocities());
      if (System.nanoTime() - start > 2_000_000_000L) {
        errorSum += drive.getPose().getTranslation().getDistance(truth.getTranslation());
        errorSamples++;
      }
      Thread.sleep(20);
    }

    double averageError = errorSum / errorSamples;
    System.out.printf(
        "SIMTEST vision: started 1.00 m off, averaged %.3f m off after 2 s%n", averageError);
    assertEquals(0.0, averageError, 0.08, "average pose error after vision, meters");
  }

  @Test
  void hardAccelerationDoesNotBrownOut() throws InterruptedException {
    double restingVolts = simWorld.getBatteryVolts();
    int brownoutsBefore = simWorld.getBrownoutCount();
    double minVolts = restingVolts;
    double peakAmps = 0.0;
    // Full speed from a standstill: the moment that browns robots out.
    long end = System.nanoTime() + 600_000_000L;
    while (System.nanoTime() < end) {
      DriverStationSim.notifyNewData();
      drive.periodic();
      drive.runVelocity(new ChassisVelocities(DriveConstants.MAX_LINEAR_SPEED, 0.0, 0.0));
      minVolts = Math.min(minVolts, simWorld.getBatteryVolts());
      peakAmps = Math.max(peakAmps, simWorld.getTotalCurrentAmps());
      Thread.sleep(5);
    }
    run(new ChassisVelocities(), 1.0);
    System.out.printf(
        "SIMTEST battery: resting %.2f V, min %.2f V at peak %.0f A, after stop %.2f V%n",
        restingVolts, minVolts, peakAmps, simWorld.getBatteryVolts());

    assertTrue(minVolts < restingVolts - 2.0, "launching should visibly sag the battery");
    assertEquals(
        brownoutsBefore, simWorld.getBrownoutCount(), "the supply limit should prevent it");
    assertEquals(restingVolts, simWorld.getBatteryVolts(), 0.1, "voltage recovers once stopped");
  }

  @Test
  void driveToPoseArrivesAndFinishes() throws InterruptedException {
    var target = AlignTargets.nearestTag(new Pose2d(8.0, 4.0, Rotation2d.ZERO)).orElseThrow();
    // Start off to one side and turned 90 degrees, so it has to translate and rotate at once.
    var start =
        new Pose2d(
            target.getTranslation().plus(new Translation2d(-1.5, 1.2)),
            target.getRotation().plus(Rotation2d.CCW_90DEG));
    driveSim.resetTruePose(start);
    drive.setPose(start);
    run(new ChassisVelocities(), 0.3);

    var scheduler = Scheduler.getDefault();
    var command = DriveCommands.driveToPose(drive, () -> target);
    scheduler.schedule(command);
    long startNanos = System.nanoTime();
    while (scheduler.isScheduledOrRunning(command) && System.nanoTime() - startNanos < 5e9) {
      DriverStationSim.notifyNewData();
      drive.periodic();
      scheduler.run();
      Thread.sleep(20);
    }
    double seconds = (System.nanoTime() - startNanos) / 1e9;
    run(new ChassisVelocities(), 0.3);

    var pose = drive.getPose();
    var truth = driveSim.getTruePose();
    System.out.printf(
        "SIMTEST driveToPose: %.2f s, odometry off by %.3f m / %.2f deg, truth off by %.3f m%n",
        seconds,
        pose.getTranslation().getDistance(target.getTranslation()),
        pose.getRotation().minus(target.getRotation()).getDegrees(),
        truth.getTranslation().getDistance(target.getTranslation()));
    assertTrue(seconds < 5.0, "should finish on its own");
    // Finishes inside 2 cm / 2 degrees, then coasts a little while stopping.
    assertEquals(0.0, pose.getTranslation().getDistance(target.getTranslation()), 0.03);
    assertEquals(0.0, pose.getRotation().minus(target.getRotation()).getDegrees(), 3.0);
    // Odometry only counts how far the wheels roll. While turning and driving at once the tread
    // slides sideways a little (more in the sim than on real tread, which grips stiffer), and
    // odometry can't see that, so it ends a few centimeters from the truth.
    assertEquals(0.0, truth.getTranslation().getDistance(target.getTranslation()), 0.06);
  }

  @Test
  void odometryTracksTruthWhileSteering() throws InterruptedException {
    run(new ChassisVelocities(), 1.0);

    // Drive at 1 m/s while the travel direction sweeps one turn per second, so the modules steer
    // continuously. Uncompensated, the bevel coupling makes the wheels roll about 0.18 m/s slower
    // or faster than odometry thinks, and the path lengths split by roughly 0.35 m over 2 s.
    double odometryPath = 0.0;
    double truePath = 0.0;
    var lastOdometry = drive.getPose();
    var lastTrue = driveSim.getTruePose();
    long startNanos = System.nanoTime();
    while (System.nanoTime() - startNanos < 2_000_000_000L) {
      double angle = 2 * Math.PI * (System.nanoTime() - startNanos) / 1e9;
      DriverStationSim.notifyNewData();
      drive.periodic();
      drive.runVelocity(new ChassisVelocities(Math.cos(angle), Math.sin(angle), 0.0));
      var odometry = drive.getPose();
      var truth = driveSim.getTruePose();
      odometryPath += odometry.getTranslation().getDistance(lastOdometry.getTranslation());
      truePath += truth.getTranslation().getDistance(lastTrue.getTranslation());
      lastOdometry = odometry;
      lastTrue = truth;
      Thread.sleep(20);
    }

    System.out.println("SIMTEST path odometry " + odometryPath + " truth " + truePath);
    assertEquals(truePath, odometryPath, 0.08, "odometry path length vs truth");
  }

  @Test
  void drivesStraightThenSpins() throws InterruptedException {
    // Point the wheels forward first with a crawl. Earlier tests leave them at odd angles, and
    // launching while they're still turning makes misaligned wheels push sideways (kS adds a
    // fixed push to even tiny wheel speeds), which yaws the robot a few degrees.
    run(new ChassisVelocities(0.01, 0.0, 0.0), 0.5);
    run(new ChassisVelocities(), 0.5);
    var start = drive.getPose();

    run(new ChassisVelocities(1.0, 0.0, 0.0), 2.0);
    // Measured from where the robot started, in its own frame: forward is +x, sideways is y.
    var moved = drive.getPose().relativeTo(start);
    System.out.println(
        "SIMTEST after drive: moved " + moved + " speeds " + drive.getChassisVelocities());
    // About 2 m less the acceleration ramp. Wide bounds: this checks direction and scale, not
    // tuning.
    assertEquals(1.9, moved.getX(), 0.25, "forward travel");
    assertEquals(0.0, moved.getY(), 0.05, "sideways drift");
    assertEquals(0.0, moved.getRotation().getDegrees(), 1.0, "turned while driving straight");

    run(new ChassisVelocities(), 0.5);
    double yawStart = drive.getRotation().getRadians();
    run(new ChassisVelocities(0.0, 0.0, Math.PI / 2), 1.0);
    run(new ChassisVelocities(), 0.5);
    double yawDelta = drive.getRotation().getRadians() - yawStart;
    System.out.println("SIMTEST yaw delta rad: " + yawDelta);
    assertEquals(Math.PI / 2, yawDelta, 0.25, "yaw after 1 s at pi/2 rad/s");
  }
}
