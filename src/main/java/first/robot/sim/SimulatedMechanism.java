package first.robot.sim;

/**
 * A physics model for one mechanism. Implementations read what their simulated motor controllers
 * are applying, advance the model by {@code dtSeconds}, and write the result back into the
 * simulated sensors.
 *
 * <p>{@link SimWorld} calls this from its own thread, not the main robot loop.
 */
public interface SimulatedMechanism {
  /**
   * @param batteryVolts what the battery is supplying right now, after sag; give this to every
   *     simulated device as its supply voltage
   * @return current this mechanism drew from the battery during the step, in amps
   */
  double update(double dtSeconds, double batteryVolts);
}
