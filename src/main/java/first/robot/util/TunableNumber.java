package first.robot.util;

import first.robot.Constants;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * A number that can be changed from the dashboard while tuning, and a plain constant otherwise.
 *
 * <p>In tuning mode it shows up in NetworkTables under "/Tuning/" + key. It's backed by
 * AdvantageKit's LoggedNetworkNumber, which records every value as a logged input, so replaying a
 * log reproduces the gain changes made during that run.
 */
public class TunableNumber implements DoubleSupplier {
  private final double defaultValue;
  // Null outside tuning mode, so nothing is published.
  private final LoggedNetworkNumber dashboardNumber;

  public TunableNumber(String key, double defaultValue) {
    this.defaultValue = defaultValue;
    dashboardNumber =
        Constants.TUNING_MODE ? new LoggedNetworkNumber("/Tuning/" + key, defaultValue) : null;
  }

  public double get() {
    return dashboardNumber != null ? dashboardNumber.get() : defaultValue;
  }

  @Override
  public double getAsDouble() {
    return get();
  }
}
