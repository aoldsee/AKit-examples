package first.robot.sim;

import com.ctre.phoenix6.sim.ChassisReference;

/** Small helpers shared by the Phoenix-backed mechanism models. */
public final class PhoenixSimUtil {
  private PhoenixSimUtil() {}

  /**
   * Phoenix sim states report and accept values in the device's own frame unless told how it's
   * mounted. Pass the same invert flag the device was configured with.
   */
  public static ChassisReference orientation(boolean inverted) {
    return inverted
        ? ChassisReference.Clockwise_Positive
        : ChassisReference.CounterClockwise_Positive;
  }

  /**
   * Crude friction: voltage below the threshold produces no motion, and above it the threshold is
   * subtracted. Good enough to make kS show up in characterization.
   */
  public static double applyFriction(double motorVolts, double frictionVolts) {
    if (Math.abs(motorVolts) < frictionVolts) {
      return 0.0;
    }
    return motorVolts - Math.copySign(frictionVolts, motorVolts);
  }
}
