package first.robot.subsystems.drive;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

/**
 * Simulated module. Everything a real module does (device configs, Phoenix closed loops, coupling
 * compensation, high-rate odometry) is inherited unchanged from {@link ModuleIOTalonFX}. The only
 * addition is handing this module's simulated devices to {@link SwerveDriveSim}, which plays the
 * part of the physical module behind them.
 */
public class ModuleIOTalonFXSim extends ModuleIOTalonFX {
  public ModuleIOTalonFXSim(
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
          constants,
      SwerveDriveSim driveSim) {
    super(constants);
    driveSim.addModule(
        constants, driveTalon.getSimState(), turnTalon.getSimState(), cancoder.getSimState());
  }
}
