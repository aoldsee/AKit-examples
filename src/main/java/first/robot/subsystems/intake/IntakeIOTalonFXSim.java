package first.robot.subsystems.intake;

import org.wpilib.simulation.DIOSim;

/**
 * Simulated intake. Everything the real intake does is inherited from {@link IntakeIOTalonFX}. The
 * only addition is handing the simulated motor and sensor to {@link IntakeSim}.
 */
public class IntakeIOTalonFXSim extends IntakeIOTalonFX {
  public IntakeIOTalonFXSim(IntakeSim intakeSim) {
    super();
    intakeSim.attach(motor.getSimState(), new DIOSim(sensor));
  }
}
