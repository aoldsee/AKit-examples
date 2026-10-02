package first.robot.sim;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.sim.ChassisReference;

/** Small helpers shared by the Phoenix-backed mechanism models. */
public final class PhoenixSimUtil {
  private PhoenixSimUtil() {}

  /**
   * Phoenix sim states report and accept values in the device's own frame unless told how it's
   * mounted. Pass the same direction the motor was configured with.
   */
  public static ChassisReference orientation(InvertedValue direction) {
    return direction == InvertedValue.Clockwise_Positive
        ? ChassisReference.Clockwise_Positive
        : ChassisReference.CounterClockwise_Positive;
  }

  /** The same, for a CANcoder's configured direction. */
  public static ChassisReference orientation(SensorDirectionValue direction) {
    return direction == SensorDirectionValue.Clockwise_Positive
        ? ChassisReference.Clockwise_Positive
        : ChassisReference.CounterClockwise_Positive;
  }

  /**
   * Friction as a voltage, the way characterization measures it (kS): a constant push of {@code
   * frictionVolts} against the motion. Returns the voltage left over to accelerate the mechanism.
   *
   * <ul>
   *   <li>Stopped: friction holds it still until the voltage beats it (static friction).
   *   <li>Moving: friction pushes against the direction of motion, whichever way the voltage
   *       points, so it slows a coasting or braking mechanism too.
   * </ul>
   *
   * Call {@link #stoppedByFriction} after stepping the model, so friction can stop the mechanism
   * but never push it backward.
   */
  public static double applyFriction(double volts, double velocity, double frictionVolts) {
    if (velocity == 0.0) {
      return Math.abs(volts) <= frictionVolts ? 0.0 : volts - Math.copySign(frictionVolts, volts);
    }
    return volts - Math.copySign(frictionVolts, velocity);
  }

  /**
   * True if a step carried the velocity through zero while the voltage couldn't beat friction. The
   * model doesn't know friction can only stop motion, so the caller should set the velocity to
   * exactly zero.
   */
  public static boolean stoppedByFriction(
      double velocityBefore, double velocityAfter, double volts, double frictionVolts) {
    return velocityBefore * velocityAfter < 0 && Math.abs(volts) <= frictionVolts;
  }
}
