package first.robot.subsystems.arm;

/**
 * Simulated arm. Everything the real arm does (configs, fused CANcoder, Motion Magic, gravity
 * feedforward, soft limits) is inherited unchanged from {@link ArmIOTalonFX}. The only addition is
 * handing the simulated devices to {@link ArmSim}, which plays the physical arm behind them.
 */
public class ArmIOTalonFXSim extends ArmIOTalonFX {
  public ArmIOTalonFXSim(ArmSim armSim) {
    super();
    armSim.attach(motor.getSimState(), encoder.getSimState());
  }
}
