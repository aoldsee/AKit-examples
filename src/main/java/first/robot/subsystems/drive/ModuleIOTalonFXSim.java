package first.robot.subsystems.drive;

/**
 * Simulated module. Everything a real module does (device configs, Phoenix closed loops, coupling
 * compensation, high-rate odometry) is inherited unchanged from {@link ModuleIOTalonFX}. The only
 * addition is handing this module's simulated devices to {@link SwerveDriveSim}, which plays the
 * part of the physical module behind them.
 */
public class ModuleIOTalonFXSim extends ModuleIOTalonFX {
  public ModuleIOTalonFXSim(DriveConstants.ModuleConfig module, SwerveDriveSim driveSim) {
    super(module);
    driveSim.addModule(
        module, driveTalon.getSimState(), turnTalon.getSimState(), cancoder.getSimState());
  }
}
