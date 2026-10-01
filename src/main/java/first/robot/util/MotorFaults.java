package first.robot.util;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import org.wpilib.util.Alert;
import org.wpilib.util.Alert.Level;

/**
 * Problems a Talon FX has reported since the robot code started.
 *
 * <p>A Talon keeps two sets of fault flags. Live faults are true only while the problem is
 * happening. Sticky faults stay true until cleared, so a brownout that lasted a few milliseconds
 * between status frames is still visible afterward. These are the sticky ones, cleared when the
 * robot code starts, so each flag means "this happened at least once since startup".
 *
 * <p>The two current-limit flags are the exception: they're live, because a motor hitting its limit
 * is normal, and what matters is when and how often. They're logged but don't raise alerts.
 *
 * <p>The IO layer reads these with {@link Signals} and stores them in its inputs, so they're logged
 * and replayed like any other sensor. The mechanism turns them into dashboard alerts with {@link
 * Alerts}.
 *
 * @param hardware the motor controller found an internal hardware problem
 * @param overheating the motor or its controller got too hot and may have limited its output
 * @param undervoltage the supply voltage dropped close to brownout
 * @param brownout the supply voltage dropped so low the motor shut its output off
 * @param rebootedWhileEnabled the controller restarted while the robot was enabled, which almost
 *     always means a loose power connection
 * @param unlicensed a Phoenix Pro feature (FOC, fused CANcoder) is in use without a license, so the
 *     motor is running in a fallback mode
 * @param sensorProblem the remote CANcoder's data stopped being trusted, or a fused CANcoder
 *     disagreed with the motor's own sensor (often a slipping belt or loose magnet)
 * @param statorCurrentLimited the stator current limit is active right now
 * @param supplyCurrentLimited the supply current limit is active right now
 */
public record MotorFaults(
    boolean hardware,
    boolean overheating,
    boolean undervoltage,
    boolean brownout,
    boolean rebootedWhileEnabled,
    boolean unlicensed,
    boolean sensorProblem,
    boolean statorCurrentLimited,
    boolean supplyCurrentLimited) {
  public static final MotorFaults NONE =
      new MotorFaults(false, false, false, false, false, false, false, false, false);

  /** Fault frames are slow on purpose. Faults don't need to be read 50 times a second. */
  private static final double UPDATE_FREQUENCY_HZ = 4.0;

  /** Reads one Talon's fault flags. Create it in the IO constructor. */
  public static final class Signals {
    private final StatusSignal<Boolean> hardware;
    private final StatusSignal<Boolean> deviceTemp;
    private final StatusSignal<Boolean> processorTemp;
    private final StatusSignal<Boolean> undervoltage;
    private final StatusSignal<Boolean> brownout;
    private final StatusSignal<Boolean> bootDuringEnable;
    private final StatusSignal<Boolean> unlicensed;
    private final StatusSignal<Boolean> remoteSensorInvalid;
    private final StatusSignal<Boolean> fusedSensorOutOfSync;
    private final StatusSignal<Boolean> statorLimit;
    private final StatusSignal<Boolean> supplyLimit;
    private final BaseStatusSignal[] all;

    public Signals(TalonFX talon) {
      // Start from a clean slate, so a fault left over from before this boot (a brownout while
      // the robot sat on the cart, say) doesn't raise an alert.
      PhoenixUtil.tryUntilOk(5, () -> talon.clearStickyFaults(0.25));

      // false skips the refresh on lookup, for the same reason as the IO's other signals.
      hardware = talon.getStickyFault_Hardware(false);
      deviceTemp = talon.getStickyFault_DeviceTemp(false);
      processorTemp = talon.getStickyFault_ProcTemp(false);
      undervoltage = talon.getStickyFault_Undervoltage(false);
      brownout = talon.getStickyFault_BridgeBrownout(false);
      bootDuringEnable = talon.getStickyFault_BootDuringEnable(false);
      unlicensed = talon.getStickyFault_UnlicensedFeatureInUse(false);
      remoteSensorInvalid = talon.getStickyFault_RemoteSensorDataInvalid(false);
      fusedSensorOutOfSync = talon.getStickyFault_FusedSensorOutOfSync(false);
      statorLimit = talon.getFault_StatorCurrLimit(false);
      supplyLimit = talon.getFault_SupplyCurrLimit(false);
      all =
          new BaseStatusSignal[] {
            hardware,
            deviceTemp,
            processorTemp,
            undervoltage,
            brownout,
            bootDuringEnable,
            unlicensed,
            remoteSensorInvalid,
            fusedSensorOutOfSync,
            statorLimit,
            supplyLimit
          };
      // Must be set before the IO calls optimizeBusUtilization, which turns off every frame that
      // wasn't given a rate.
      BaseStatusSignal.setUpdateFrequencyForAll(UPDATE_FREQUENCY_HZ, all);
      // At this slow rate the first frame can take a quarter second to arrive, and the first
      // robot loop would report it missing. Waiting for it here costs a moment at startup.
      BaseStatusSignal.waitForAll(2.0 / UPDATE_FREQUENCY_HZ, all);
    }

    /** Refreshes the flags and returns them. Call from updateInputs. */
    public MotorFaults read() {
      // A slow fault frame isn't a disconnect, so the status here is ignored; the IO's other
      // signals already decide whether the motor is connected.
      BaseStatusSignal.refreshAll(all);
      return new MotorFaults(
          hardware.getValue(),
          deviceTemp.getValue() || processorTemp.getValue(),
          undervoltage.getValue(),
          brownout.getValue(),
          bootDuringEnable.getValue(),
          unlicensed.getValue(),
          remoteSensorInvalid.getValue() || fusedSensorOutOfSync.getValue(),
          statorLimit.getValue(),
          supplyLimit.getValue());
    }
  }

  /** Dashboard alerts for one motor's faults. Create it in the mechanism. */
  public static final class Alerts {
    private final Alert hardware;
    private final Alert overheating;
    private final Alert undervoltage;
    private final Alert brownout;
    private final Alert rebootedWhileEnabled;
    private final Alert unlicensed;
    private final Alert sensorProblem;

    /**
     * @param group the alert group, usually the mechanism's name
     * @param motor which motor, both in the alert ID and the message (e.g. "Module0/Drive")
     */
    public Alerts(String group, String motor) {
      // Each message says what to check, because that's what someone reading it in the pits
      // needs. Overheating and low voltage can clear up on their own, so they're less urgent.
      hardware =
          alert(
              group,
              motor,
              "Hardware",
              "hardware fault. Power cycle; replace if it repeats.",
              Level.HIGH);
      overheating =
          alert(
              group,
              motor,
              "Overheating",
              "overheated. Let it cool; check for binding.",
              Level.MEDIUM);
      undervoltage =
          alert(
              group,
              motor,
              "Undervoltage",
              "saw low voltage. Check the battery and wiring.",
              Level.MEDIUM);
      brownout =
          alert(
              group,
              motor,
              "Brownout",
              "browned out. Check the battery and current limits.",
              Level.HIGH);
      rebootedWhileEnabled =
          alert(
              group,
              motor,
              "RebootedWhileEnabled",
              "rebooted while enabled. Check its power connection.",
              Level.HIGH);
      unlicensed =
          alert(
              group,
              motor,
              "Unlicensed",
              "uses a Phoenix Pro feature without a license.",
              Level.HIGH);
      sensorProblem =
          alert(
              group,
              motor,
              "SensorProblem",
              "lost trust in its CANcoder. Check the magnet and for a slipping belt.",
              Level.HIGH);
    }

    private static Alert alert(String group, String motor, String id, String problem, Level level) {
      return new Alert(group, motor + "/" + id, motor + " " + problem, level);
    }

    public void update(MotorFaults faults) {
      hardware.set(faults.hardware());
      overheating.set(faults.overheating());
      undervoltage.set(faults.undervoltage());
      brownout.set(faults.brownout());
      rebootedWhileEnabled.set(faults.rebootedWhileEnabled());
      unlicensed.set(faults.unlicensed());
      sensorProblem.set(faults.sensorProblem());
    }
  }
}
