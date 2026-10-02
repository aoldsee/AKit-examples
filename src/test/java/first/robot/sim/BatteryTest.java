package first.robot.sim;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

/** The battery model, checked against numbers worked out by hand. */
class BatteryTest {
  @Test
  void restingVoltageFallsAsTheBatteryDrains() {
    assertEquals(Battery.FULL_VOLTS, Battery.restingVoltage(1.0), 1e-9);
    assertEquals(Battery.EMPTY_VOLTS, Battery.restingVoltage(0.0), 1e-9);
    assertEquals((Battery.FULL_VOLTS + Battery.EMPTY_VOLTS) / 2, Battery.restingVoltage(0.5), 1e-9);
  }

  @Test
  void drawingCurrentUsesUpCharge() {
    var battery = new Battery(18.0, 1.0);
    // 36 A for 450 s (an eighth of an hour) is 4.5 Ah, a quarter of 18 Ah.
    battery.draw(36.0, 450.0);
    assertEquals(4.5, battery.getAmpHoursUsed(), 1e-9);
    assertEquals(0.75, battery.getStateOfCharge(), 1e-9);

    // Can't go below empty.
    battery.draw(1000.0, 3600.0);
    assertEquals(0.0, battery.getStateOfCharge(), 1e-9);
  }

  @Test
  void defaultBatteryNeverRunsDown() {
    var battery = new Battery();
    battery.draw(1000.0, 3600.0);
    assertEquals(1.0, battery.getStateOfCharge(), 1e-9);
    assertEquals(Battery.FULL_VOLTS, battery.getRestingVoltage(), 1e-9);
  }

  @Test
  void startingChargeSetsTheStartingVoltage() {
    assertEquals(Battery.restingVoltage(0.5), new Battery(18.0, 0.5).getRestingVoltage(), 1e-9);
  }

  @Test
  void currentSagsTheVoltageThroughTheResistance() {
    var battery = new Battery();
    // 300 A through 0.020 ohms costs 6 V.
    double sag = battery.getRestingVoltage() - battery.getLoadedVoltage(300.0);
    assertEquals(300.0 * Battery.RESISTANCE_OHMS, sag, 1e-9);
  }

  @Test
  void aNearlyFlatBatteryBrownsOutAtLowerCurrent() {
    var full = new Battery(18.0, 1.0);
    var nearlyFlat = new Battery(18.0, 0.1);
    // The current that pulls each one down to the brownout line.
    double fullLimit =
        (full.getRestingVoltage() - Battery.BROWNOUT_VOLTS) / Battery.RESISTANCE_OHMS;
    double flatLimit =
        (nearlyFlat.getRestingVoltage() - Battery.BROWNOUT_VOLTS) / Battery.RESISTANCE_OHMS;
    assertTrue(flatLimit < fullLimit);
    assertTrue(
        nearlyFlat.getLoadedVoltage((fullLimit + flatLimit) / 2) < Battery.BROWNOUT_VOLTS,
        "a current a full battery handles browns out a nearly flat one");
  }

  @Test
  void nonsenseArgumentsAreRejected() {
    assertThrows(IllegalArgumentException.class, () -> new Battery(0.0, 1.0));
    assertThrows(IllegalArgumentException.class, () -> new Battery(18.0, 1.5));
  }
}
