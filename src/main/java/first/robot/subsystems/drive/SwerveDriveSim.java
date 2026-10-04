package first.robot.subsystems.drive;

import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import first.robot.Constants;
import first.robot.sim.SimulatedMechanism;
import java.util.ArrayList;
import java.util.List;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Twist2d;
import org.wpilib.math.util.Units;

/**
 * Physics for the whole robot, treated as one body that the four wheels push on.
 *
 * <p>Each step:
 *
 * <ol>
 *   <li>Each module turns its steering.
 *   <li>Each wheel works out how hard it pushes, from its motor's voltage and how it's moving over
 *       the ground ({@link SwerveModuleSim#push}).
 *   <li>The pushes add up to a total force and a total twist (each push times its distance from the
 *       center). Newton's second law turns those into acceleration (force / mass) and spin-up
 *       (twist / moment of inertia).
 *   <li>The robot's speed and position update, and each wheel's sensors report how it moved.
 * </ol>
 *
 * <p>The robot's motion becomes the simulated Pigeon's yaw and the logged true pose. Modules and
 * the gyro join through {@link ModuleIOTalonFXSim} and {@link GyroIOPigeon2Sim}.
 */
public class SwerveDriveSim implements SimulatedMechanism {
  // FUDGE: real-world effects this sim leaves out. Set to "no effect" here. To make the sim act
  // like a particular real robot, adjust these, not the robot's constants.

  // FUDGE: how fast the simulated gyro's heading drifts, degrees per minute. Real gyros drift a
  // little. Vision pulls the heading back (through MegaTag1) whenever two or more tags are in view;
  // away from tags, the drift shows up directly in the pose.
  private static final double FUDGE_GYRO_DRIFT_DEG_PER_MIN = 0.0;

  private final List<SwerveModuleSim> modules = new ArrayList<>();
  private Pigeon2SimState gyroSim = null;

  // The robot's speed, relative to its own frame: forward and left in m/s, spin in rad/s.
  private Translation2d velocity = Translation2d.ZERO;
  private double omega = 0.0;

  private Rotation2d yaw = Rotation2d.ZERO;
  // Written by the SimWorld thread, read by the main loop. Pose2d is immutable, so volatile is
  // enough.
  private volatile Pose2d truePose = Pose2d.ZERO;
  // A reset from the main loop is handed over here and applied by update(), on the SimWorld
  // thread. Writing truePose directly could be overwritten by an update already in progress.
  private volatile Pose2d pendingReset = null;

  /** Called by {@link ModuleIOTalonFXSim}, before the SimWorld starts. */
  void addModule(
      DriveConstants.ModuleConfig module,
      TalonFXSimState driveSim,
      TalonFXSimState steerSim,
      CANcoderSimState encoderSim) {
    modules.add(new SwerveModuleSim(module, driveSim, steerSim, encoderSim));
  }

  /** Called by {@link GyroIOPigeon2Sim}, before the SimWorld starts. */
  void setGyro(Pigeon2SimState gyroSim) {
    this.gyroSim = gyroSim;
  }

  @Override
  public double update(double dtSeconds, double batteryVolts) {
    var reset = pendingReset;
    if (reset != null) {
      pendingReset = null;
      truePose = reset;
      velocity = Translation2d.ZERO;
      omega = 0.0;
    }

    for (var module : modules) {
      module.updateSteering(dtSeconds, batteryVolts);
    }

    // Add up every wheel's push into one force and one twist.
    var force = Translation2d.ZERO;
    double torque = 0.0;
    for (var module : modules) {
      var r = module.getLocation();
      // The module moves with the robot, plus the spin: a point r from the center of a body
      // spinning at omega moves at omega * r, at right angles to r.
      var moduleVelocity = velocity.plus(new Translation2d(-omega * r.getY(), omega * r.getX()));
      var push = module.push(moduleVelocity);
      force = force.plus(push);
      torque += r.getX() * push.getY() - r.getY() * push.getX();
    }

    // Newton's second law. The velocity is in the robot's own frame, which turns as the robot
    // spins, so the part of the old velocity that's now pointing sideways has to be turned back.
    var acceleration = force.div(Constants.ROBOT_MASS_KG);
    double angularAcceleration = torque / DriveConstants.ROBOT_MOI_KG_M2;
    velocity =
        velocity
            .plus(acceleration.times(dtSeconds))
            .rotateBy(Rotation2d.fromRadians(-omega * dtSeconds));
    omega += angularAcceleration * dtSeconds;
    truePose =
        truePose.plus(
            new Twist2d(velocity.getX() * dtSeconds, velocity.getY() * dtSeconds, omega * dtSeconds)
                .exp());

    for (var module : modules) {
      module.updateSensors(dtSeconds);
    }

    // The gyro measures the robot's real rotation, plus its own drift.
    yaw =
        yaw.plus(
            Rotation2d.fromRadians(omega * dtSeconds)
                .plus(Rotation2d.fromDegrees(FUDGE_GYRO_DRIFT_DEG_PER_MIN / 60.0 * dtSeconds)));
    if (gyroSim != null) {
      gyroSim.setSupplyVoltage(batteryVolts);
      gyroSim.setRawYaw(yaw.getDegrees());
      gyroSim.setAngularVelocityZ(Units.radiansToDegrees(omega));
    }

    double amps = 0.0;
    for (var module : modules) {
      amps += module.getSupplyCurrentAmps();
    }
    return amps;
  }

  /** Where the robot actually is. Compare against odometry to catch IO bugs. */
  public Pose2d getTruePose() {
    return truePose;
  }

  /**
   * Teleports the simulated robot, stopped, for when odometry is reset to a known pose. The gyro is
   * left alone because a real Pigeon doesn't know the robot was picked up.
   */
  public void resetTruePose(Pose2d pose) {
    pendingReset = pose;
  }
}
