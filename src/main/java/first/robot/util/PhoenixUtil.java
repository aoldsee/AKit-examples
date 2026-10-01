package first.robot.util;

import com.ctre.phoenix6.StatusCode;
import java.util.function.Supplier;

public final class PhoenixUtil {
  private PhoenixUtil() {}

  /**
   * Runs a Phoenix call until it returns OK or runs out of attempts. Config applies right after
   * boot can time out while the device is still enumerating.
   */
  public static void tryUntilOk(int maxAttempts, Supplier<StatusCode> command) {
    for (int i = 0; i < maxAttempts; i++) {
      if (command.get().isOK()) {
        break;
      }
    }
  }
}
