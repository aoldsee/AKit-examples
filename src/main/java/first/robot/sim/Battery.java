package first.robot.sim;

/**
 * A simple model of the robot's battery: an ideal voltage source behind a resistor. Every amp the
 * robot draws drops the voltage the electronics see by (amps * resistance). That's the whole story
 * behind most brownouts: draw enough current at once and the voltage falls below what the
 * controller needs to stay running.
 */
public final class Battery {
  // Resting voltage of a lead-acid battery at full and empty charge, roughly linear in between.
  // Measured with no load; under load, subtract current times resistance.
  public static final double FULL_VOLTS = 12.8;
  public static final double EMPTY_VOLTS = 11.8;

  /**
   * Battery plus main breaker, wiring, and connectors, in ohms. This is WPILib's default. A worn
   * battery or a loose connector raises it, and even a few extra milliohms matter at 300 A. Real
   * batteries also gain resistance as they empty; this model keeps it fixed.
   */
  public static final double RESISTANCE_OHMS = 0.020;

  /** Steady draw from the controller, radio, cameras, and sensors. A rough estimate. */
  public static final double BASE_LOAD_AMPS = 3.0;

  /**
   * SystemCore's default brownout thresholds. Below BROWNOUT_VOLTS the robot browns out, and it
   * stays that way until the voltage climbs back above RECOVERY_VOLTS. The gap keeps it from
   * flickering in and out. RobotController.setBrownoutVoltages can change both.
   */
  public static final double BROWNOUT_VOLTS = 6.75;

  public static final double RECOVERY_VOLTS = 7.25;

  /**
   * The most power a full battery can ever deliver, FULL_VOLTS^2 / (4 * RESISTANCE_OHMS), about
   * 2000 W, reached when the voltage has sagged to half. It shrinks as the battery drains. Four
   * drive motors launching hard with no supply limit can ask for several times that, and then no
   * voltage satisfies them: it just keeps falling until the robot browns out. A supply current
   * limit keeps the demand under this ceiling.
   */
  public static final double MAX_POWER_WATTS = FULL_VOLTS * FULL_VOLTS / (4 * RESISTANCE_OHMS);

  private final double capacityAh;
  private final double startingCharge;
  private double ampHoursUsed = 0.0;

  /** A full battery that never runs down, for when charge isn't what's being studied. */
  public Battery() {
    this(Double.POSITIVE_INFINITY, 1.0);
  }

  /**
   * @param capacityAh amp-hours when full. FRC batteries are rated 18 Ah, but at a gentle 20-hour
   *     discharge; at robot currents a real one gives noticeably less (Peukert's Law, which we
   *     could model but is itself only really useful in estimating constant current draw, not the
   *     extremely spiky FRC usual), so this model is quite optimistic. Double.POSITIVE_INFINITY
   *     never runs down.
   * @param startingCharge how charged it is to begin with, from 0 (flat) to 1 (full)
   */
  public Battery(double capacityAh, double startingCharge) {
    if (!(capacityAh > 0.0)) {
      throw new IllegalArgumentException("capacityAh must be positive, got " + capacityAh);
    }
    if (startingCharge < 0.0 || startingCharge > 1.0) {
      throw new IllegalArgumentException(
          "startingCharge must be between 0 and 1, got " + startingCharge);
    }
    this.capacityAh = capacityAh;
    this.startingCharge = startingCharge;
  }

  /** Records {@code amps} drawn for {@code dtSeconds}, running the battery down. */
  public void draw(double amps, double dtSeconds) {
    ampHoursUsed += amps * dtSeconds / 3600.0;
  }

  public double getAmpHoursUsed() {
    return ampHoursUsed;
  }

  /** How charged the battery is now, from 0 (flat) to 1 (full). */
  public double getStateOfCharge() {
    // An infinite capacity divides to zero here, so the charge never moves.
    return Math.clamp(startingCharge - ampHoursUsed / capacityAh, 0.0, 1.0);
  }

  /** Voltage with no load, which falls as the battery drains. */
  public double getRestingVoltage() {
    return restingVoltage(getStateOfCharge());
  }

  /** Voltage at the robot's terminals while it draws {@code totalAmps}. */
  public double getLoadedVoltage(double totalAmps) {
    return Math.max(0.0, getRestingVoltage() - totalAmps * RESISTANCE_OHMS);
  }

  /** Resting voltage at a given state of charge, from 0 to 1. */
  public static double restingVoltage(double stateOfCharge) {
    return EMPTY_VOLTS + (FULL_VOLTS - EMPTY_VOLTS) * stateOfCharge;
  }
}
