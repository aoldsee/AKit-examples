package first.robot.sim;

import first.robot.util.Battery;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import org.littletonrobotics.junction.Logger;
import org.wpilib.simulation.RoboRioSim;
import org.wpilib.system.Notifier;
import org.wpilib.system.Timer;
import org.wpilib.util.Alert;
import org.wpilib.util.Alert.Level;

/**
 * Stands in for the physical robot in simulation.
 *
 * <p>In SIM the robot code talks to Phoenix's simulated devices exactly as it would talk to real
 * ones. Something still has to play the motors, gears, and gravity on the other side of those
 * devices. Each mechanism has a small {@link SimulatedMechanism} model for that, kept next to its
 * IO, and this class steps all of them together on one fixed-rate loop.
 *
 * <p>It also plays the battery. Each step, every mechanism reports the current it drew, and the
 * total sets how far the battery voltage sags (see {@link Battery}). The next step every device
 * runs on that lower voltage, so hard acceleration really does leave less for everything else. The
 * battery also runs down as charge is used, so the same move sags it further late in a long
 * session. The voltage is published as the robot's battery voltage, so AdvantageKit logs it like it
 * would on a real robot.
 *
 * <p>Add every mechanism, then call {@link #start()} once.
 */
public final class SimWorld implements AutoCloseable {
  // 250 Hz matches the drive's odometry rate, so every odometry sample sees fresh physics.
  private static final double PERIOD_SECS = 0.004;
  // Fraction of the way to the new battery voltage per step: about a 15 ms time constant.
  private static final double VOLTAGE_RESPONSE = 0.25;

  private record Entry(String name, SimulatedMechanism mechanism) {}

  private final List<Entry> mechanisms = new ArrayList<>();
  private final Notifier notifier = new Notifier(this::step);
  private double lastTimeSecs = 0.0;

  // Written by the SimWorld thread, read by the main loop when logging.
  // Only the SimWorld thread touches the battery; the volatile copies below are for logging.
  private final Battery battery;
  private volatile double batteryVolts;
  private volatile double ampHoursUsed = 0.0;
  private volatile double stateOfCharge;
  private volatile double totalCurrentAmps = 0.0;
  private final Map<String, Double> currentAmpsByMechanism = new ConcurrentHashMap<>();
  private volatile boolean brownedOut = false;
  private volatile int brownoutCount = 0;

  private final Alert brownoutAlert =
      new Alert(
          "Sim",
          "Brownout",
          "Simulated brownout: the battery fell below "
              + Battery.BROWNOUT_VOLTS
              + " V at least once.",
          Level.HIGH);

  /** A world with a battery that never runs down. */
  public SimWorld() {
    this(new Battery());
  }

  public SimWorld(Battery battery) {
    this.battery = battery;
    batteryVolts = battery.getRestingVoltage();
    stateOfCharge = battery.getStateOfCharge();
    notifier.setName("SimWorld");
  }

  /**
   * Must be called before {@link #start()}; the list isn't safe to change while running.
   *
   * @param name shown in the logs next to this mechanism's current draw
   */
  public void add(String name, SimulatedMechanism mechanism) {
    mechanisms.add(new Entry(name, mechanism));
  }

  public void start() {
    lastTimeSecs = Timer.getMonotonicTimestamp();
    notifier.startPeriodic(PERIOD_SECS);
  }

  private void step() {
    // Notifier jitter is a few hundred microseconds, so integrate over measured time.
    double now = Timer.getMonotonicTimestamp();
    double dt = now - lastTimeSecs;
    lastTimeSecs = now;

    double total = Battery.BASE_LOAD_AMPS;
    for (var entry : mechanisms) {
      double amps = entry.mechanism().update(dt, batteryVolts);
      currentAmpsByMechanism.put(entry.name(), amps);
      total += amps;
    }
    totalCurrentAmps = total;

    // Counting amp-hours drawn is how the battery runs down over a match. An 18 Ah battery at a
    // 50 A average loses about 12% of its charge in a 2.5 minute match.
    battery.draw(total, dt);
    ampHoursUsed = battery.getAmpHoursUsed();
    stateOfCharge = battery.getStateOfCharge();

    // Motor controllers pull more current when the voltage drops, so jumping straight to the new
    // voltage overshoots and the sim rings back and forth. Moving part of the way each step
    // settles on the right voltage instead, or keeps falling if the demand is more than the
    // battery can ever supply (see Battery.MAX_POWER_WATTS). Real controllers smooth it the same
    // way, with the capacitors on their inputs.
    double target = battery.getLoadedVoltage(total);
    batteryVolts += VOLTAGE_RESPONSE * (target - batteryVolts);
    RoboRioSim.setVInVoltage(batteryVolts);

    // The simulator never browns out on its own, so track it here with the same thresholds and
    // the same gap between entering and leaving that SystemCore uses.
    if (!brownedOut && batteryVolts < Battery.BROWNOUT_VOLTS) {
      brownedOut = true;
      brownoutCount++;
    } else if (brownedOut && batteryVolts > Battery.RECOVERY_VOLTS) {
      brownedOut = false;
    }
  }

  public double getBatteryVolts() {
    return batteryVolts;
  }

  public double getTotalCurrentAmps() {
    return totalCurrentAmps;
  }

  public int getBrownoutCount() {
    return brownoutCount;
  }

  /** Call from the main robot loop; AdvantageKit logging isn't safe from the SimWorld thread. */
  public void logOutputs() {
    Logger.recordOutput("Sim/Battery/Volts", batteryVolts);
    Logger.recordOutput("Sim/Battery/TotalCurrentAmps", totalCurrentAmps);
    Logger.recordOutput("Sim/Battery/AmpHoursUsed", ampHoursUsed);
    Logger.recordOutput("Sim/Battery/StateOfChargePercent", 100.0 * stateOfCharge);
    currentAmpsByMechanism.forEach(
        (name, amps) -> Logger.recordOutput("Sim/Battery/CurrentAmps/" + name, amps));
    Logger.recordOutput("Sim/Battery/BrownedOut", brownedOut);
    Logger.recordOutput("Sim/Battery/BrownoutCount", brownoutCount);
    brownoutAlert.set(brownoutCount > 0);
  }

  @Override
  public void close() {
    notifier.close();
  }
}
