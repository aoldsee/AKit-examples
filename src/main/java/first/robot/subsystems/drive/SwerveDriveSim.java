package first.robot.subsystems.drive;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import first.robot.sim.SimulatedMechanism;
import java.util.ArrayList;
import java.util.List;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.math.util.Units;

/**
 * Physics for the whole drivetrain: steps each {@link SwerveModuleSim}, then works out how the
 * robot moved from the wheels' true motion. That motion becomes the simulated Pigeon's yaw and the
 * logged true pose.
 *
 * <p>Modules and the gyro join through {@link ModuleIOTalonFXSim} and {@link GyroIOPigeon2Sim}.
 * Wheels never slip here, so the chassis motion is exact kinematics. Swap in a real chassis model
 * before trusting sim for anything traction-related.
 */
public class SwerveDriveSim implements SimulatedMechanism {
  private final List<SwerveModuleSim> modules = new ArrayList<>();
  private SwerveDriveKinematics kinematics = null;
  private Pigeon2SimState gyroSim = null;

  private Rotation2d yaw = Rotation2d.ZERO;
  // Written by the SimWorld thread, read by the main loop. Pose2d is immutable, so volatile is
  // enough.
  private volatile Pose2d truePose = Pose2d.ZERO;
  // A reset from the main loop is handed over here and applied by update(), on the SimWorld
  // thread. Writing truePose directly could be overwritten by an update already in progress.
  private volatile Pose2d pendingReset = null;

  /** Called by {@link ModuleIOTalonFXSim}, before the SimWorld starts. */
  void addModule(
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
          constants,
      TalonFXSimState driveSim,
      TalonFXSimState steerSim,
      CANcoderSimState encoderSim) {
    modules.add(new SwerveModuleSim(constants, driveSim, steerSim, encoderSim));
  }

  /** Called by {@link GyroIOPigeon2Sim}, before the SimWorld starts. */
  void setGyro(Pigeon2SimState gyroSim) {
    this.gyroSim = gyroSim;
  }

  @Override
  public double update(double dtSeconds, double batteryVolts) {
    if (kinematics == null) {
      // Built on the first step, once every module has registered. Using the registered modules'
      // own locations keeps velocities and translations in the same order by construction.
      var translations = new Translation2d[modules.size()];
      for (int i = 0; i < translations.length; i++) {
        translations[i] = modules.get(i).getLocation();
      }
      kinematics = new SwerveDriveKinematics(translations);
    }

    var velocities = new SwerveModuleVelocity[modules.size()];
    for (int i = 0; i < velocities.length; i++) {
      velocities[i] = modules.get(i).update(dtSeconds, batteryVolts);
    }

    var reset = pendingReset;
    if (reset != null) {
      pendingReset = null;
      truePose = reset;
    }
    var chassis = kinematics.toChassisVelocities(velocities);
    truePose = truePose.plus(chassis.toTwist2d(dtSeconds).exp());
    yaw = yaw.plus(Rotation2d.fromRadians(chassis.omega * dtSeconds));
    if (gyroSim != null) {
      gyroSim.setSupplyVoltage(batteryVolts);
      gyroSim.setRawYaw(yaw.getDegrees());
      gyroSim.setAngularVelocityZ(Units.radiansToDegrees(chassis.omega));
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
   * Teleports the simulated robot, for when odometry is reset to a known pose. The gyro is left
   * alone because a real Pigeon doesn't know the robot was picked up.
   */
  public void resetTruePose(Pose2d pose) {
    pendingReset = pose;
  }
}
