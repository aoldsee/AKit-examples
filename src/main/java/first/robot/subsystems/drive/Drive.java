package first.robot.subsystems.drive;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Mechanism;
import org.wpilib.driverstation.RobotState;
import org.wpilib.math.estimator.SwerveDrivePoseEstimator;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Twist2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N3;
import org.wpilib.util.Alert;
import org.wpilib.util.Alert.Level;

/**
 * Swerve drive with high-rate odometry.
 *
 * <p>Odometry works out where the robot is by adding up small wheel movements. The main loop runs
 * every 20 ms, and at 4.5 m/s the robot covers 9 cm in that time. A lot can change in 9 cm: the
 * wheels can turn to a new angle partway through, and readings taken at slightly different times
 * don't quite agree. So {@link PhoenixOdometryThread} samples the wheels and gyro together every 4
 * ms instead, and each loop this class replays every sample it collected, oldest first.
 *
 * <p>The samples feed a pose estimator, which blends wheel odometry with vision. Every sample and
 * every camera frame carries the time it was measured, so a camera frame that arrives 50 ms late
 * still corrects the pose from 50 ms ago, not the current one.
 *
 * <p>Commands v3 mechanisms have no periodic hook, so the robot must call {@link #periodic()} every
 * loop before running the scheduler.
 */
public class Drive implements Mechanism {
  /**
   * Held while reading inputs so the odometry thread can't add samples mid-read. Without it, one
   * module could read 5 samples and the next 6, and the pose estimator would get mismatched data.
   */
  static final Lock odometryLock = new ReentrantLock();

  private static final double LOOP_PERIOD_SECS = 0.02;

  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4];
  private final Alert gyroDisconnectedAlert =
      new Alert(
          "Drive",
          "GyroDisconnected",
          "Disconnected gyro, using kinematics as fallback.",
          Level.HIGH);

  private final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(DriveConstants.MODULE_TRANSLATIONS);
  // Integrated separately from the pose estimator so a gyro dropout falls back to wheel-derived
  // heading without a jump.
  private Rotation2d rawGyroRotation = Rotation2d.ZERO;
  private final SwerveModulePosition[] lastModulePositions =
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private final SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(
          kinematics,
          rawGyroRotation,
          lastModulePositions,
          Pose2d.ZERO,
          DriveConstants.ODOMETRY_STD_DEVS,
          // Placeholder; every vision measurement brings its own (see Vision).
          VecBuilder.fill(0.9, 0.9, 0.9));

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    this.gyroIO = gyroIO;
    ModuleIO[] moduleIOs = {flModuleIO, frModuleIO, blModuleIO, brModuleIO};
    for (int i = 0; i < modules.length; i++) {
      modules[i] = new Module(moduleIOs[i], i);
    }

    // Starts only if a hardware IO registered signals, so replay never spawns the thread.
    PhoenixOdometryThread.getInstance().start();
  }

  public void periodic() {
    odometryLock.lock();
    try {
      gyroIO.updateInputs(gyroInputs);
      Logger.processInputs("Drive/Gyro", gyroInputs);
      for (var module : modules) {
        module.periodic();
      }
    } finally {
      odometryLock.unlock();
    }

    // Disabled robots can't move anyway, but commanding stop keeps the motor controllers from
    // resuming an old request the moment the robot enables. Empty setpoint logs keep the log
    // viewer from showing the last setpoints as if they were still being sent.
    if (RobotState.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleVelocity[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleVelocity[] {});
    }

    // Every module and the gyro are sampled together by the odometry thread, so module 0's
    // timestamps index all of them. Each pass through this loop is one 4 ms snapshot.
    double[] sampleTimestamps = modules[0].getOdometryTimestamps();
    for (int i = 0; i < sampleTimestamps.length; i++) {
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int m = 0; m < 4; m++) {
        modulePositions[m] = modules[m].getOdometryPositions()[i];
        // How far each wheel moved since the last sample. Only the gyro fallback below uses
        // these; the pose estimator works from total positions.
        moduleDeltas[m] =
            new SwerveModulePosition(
                modulePositions[m].distance - lastModulePositions[m].distance,
                modulePositions[m].angle);
        lastModulePositions[m] = modulePositions[m];
      }

      if (gyroInputs.connected) {
        rawGyroRotation = gyroInputs.odometryYawPositions[i];
      } else {
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
    }

    gyroDisconnectedAlert.set(!gyroInputs.connected);
  }

  /** Robot-relative velocities in m/s and rad/s. */
  public void runVelocity(ChassisVelocities velocities) {
    // Discretizing compensates for the robot rotating during the loop period, which would
    // otherwise curve straight-line translation while spinning.
    ChassisVelocities discrete = velocities.discretize(LOOP_PERIOD_SECS);
    // If any wheel would need more than top speed, slow every wheel by the same factor. The robot
    // keeps the requested direction and spin, just slower; clipping only the fast wheels would
    // change the direction instead.
    SwerveModuleVelocity[] setpoints =
        SwerveDriveKinematics.desaturateWheelVelocities(
            kinematics.toSwerveModuleVelocities(discrete), DriveConstants.MAX_LINEAR_SPEED);

    Logger.recordOutput("SwerveStates/Setpoints", setpoints);
    Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discrete);

    SwerveModuleVelocity[] optimized = new SwerveModuleVelocity[4];
    for (int i = 0; i < 4; i++) {
      optimized[i] = modules[i].runSetpoint(setpoints[i]);
    }
    Logger.recordOutput("SwerveStates/SetpointsOptimized", optimized);
  }

  public void stop() {
    runVelocity(new ChassisVelocities());
  }

  /**
   * Stops with the modules in an X so the robot resists being pushed. They return to normal on the
   * next nonzero velocity request.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      // Empty only for a module at the robot center, which no swerve layout has.
      headings[i] = DriveConstants.MODULE_TRANSLATIONS[i].getAngle().orElseThrow();
    }
    // A zero velocity has no direction, so for a stopped wheel kinematics returns the last heading
    // it handed out. Overwriting those headings with the X and then asking for zero velocity
    // points every wheel into the X.
    kinematics.resetHeadings(headings);
    stop();
  }

  /** Each module's measured wheel speed (m/s) and angle, in FL, FR, BL, BR order. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  public SwerveModuleVelocity[] getModuleVelocities() {
    SwerveModuleVelocity[] velocities = new SwerveModuleVelocity[4];
    for (int i = 0; i < 4; i++) {
      velocities[i] = modules[i].getVelocity();
    }
    return velocities;
  }

  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      positions[i] = modules[i].getPosition();
    }
    return positions;
  }

  /** Robot-relative. */
  @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
  public ChassisVelocities getChassisVelocities() {
    return kinematics.toChassisVelocities(getModuleVelocities());
  }

  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  /**
   * The timestamp is when the camera took the picture, in seconds on the robot's clock. Odometry
   * samples (Timer.getMonotonicTimestamp), camera frames (NetworkTables receive time), and
   * Timer.getTimestamp all count on that same clock, so they line up. AdvantageKit just freezes
   * Timer.getTimestamp once per loop, so replay sees the same value.
   */
  public void addVisionMeasurement(
      Pose2d visionRobotPose, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs) {
    poseEstimator.addVisionMeasurement(visionRobotPose, timestampSeconds, visionMeasurementStdDevs);
  }

  // The methods below skip normal driving (kinematics, setpoint optimization) or exist only for
  // measuring the drivetrain. They're package-private, with no "public", so only the
  // characterization and systems-check commands in this package can call them. Code elsewhere
  // in the robot can't reach them by accident.

  /** Same open-loop output on every drive motor with all modules pointed forward. */
  void runCharacterization(double output) {
    for (var module : modules) {
      module.runCharacterization(output);
    }
  }

  /**
   * Sends every module the same angle and open-loop drive output, bypassing kinematics (see {@link
   * Module#runDirect}). Used by the systems check.
   */
  void runDirect(Rotation2d angle, double driveOutput) {
    for (var module : modules) {
      module.runDirect(angle, driveOutput);
    }
  }

  /** Wheel radians per module, for wheel radius characterization. */
  double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = modules[i].getWheelRadiusCharacterizationPosition();
    }
    return values;
  }

  /** Average wheel rotations/s, for feedforward characterization. */
  double getFFCharacterizationVelocity() {
    double output = 0.0;
    for (var module : modules) {
      output += module.getFFCharacterizationVelocity() / 4.0;
    }
    return output;
  }

  /** Each module's CANcoder reading, for working out encoder offsets. */
  Rotation2d[] getModuleAbsoluteAngles() {
    var angles = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      angles[i] = modules[i].getAbsoluteAngle();
    }
    return angles;
  }
}
