package first.robot.subsystems.arm;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;

import com.ctre.phoenix6.SignalLogger;
import first.robot.sim.SimWorld;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.hardware.hal.RobotMode;
import org.wpilib.math.util.Units;
import org.wpilib.simulation.DriverStationSim;

/** Runs the arm's sim IO against ArmSim in real time, through the real scheduler. */
@Tag("sim")
class ArmSimTest {
  private static SimWorld simWorld;
  private static Arm arm;
  private static final Scheduler scheduler = Scheduler.getDefault();

  @BeforeAll
  static void setup() {
    HAL.initialize();
    SignalLogger.enableAutoLogging(false);
    simWorld = new SimWorld();
    var armSim = new ArmSim();
    simWorld.add("Arm", armSim);
    arm = new Arm(new ArmIOTalonFXSim(armSim));
    arm.setDefaultCommand(arm.hold());
    simWorld.start();
    // Start disabled, like a real robot. Enabling happens in the test, after the arm has had a
    // few cycles to report where it is.
    DriverStationSim.setRobotMode(RobotMode.TELEOPERATED);
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
  }

  @AfterAll
  static void teardown() {
    simWorld.close();
  }

  private static void run(double seconds) throws InterruptedException {
    long end = System.nanoTime() + (long) (seconds * 1e9);
    while (System.nanoTime() < end) {
      DriverStationSim.notifyNewData();
      arm.periodic();
      scheduler.run();
      Thread.sleep(20);
    }
  }

  /** Runs goTo, checks it finished on its own, then checks the default command holds the angle. */
  private static void goToAndHold(double angleRad) throws InterruptedException {
    Command command = arm.goTo(angleRad);
    scheduler.schedule(command);
    // Worst case is the full 140 degree swing at 1 rot/s with 3 rot/s^2 ramps, under 1 s.
    run(2.0);
    assertFalse(scheduler.isScheduledOrRunning(command), "goTo should finish once at the goal");

    run(1.0);
    double errorDeg = Units.radiansToDegrees(arm.getPositionRad() - angleRad);
    System.out.printf(
        "SIMTEST arm goal %.1f deg, holding at %.2f deg%n",
        Units.radiansToDegrees(angleRad), Units.radiansToDegrees(arm.getPositionRad()));
    assertEquals(0.0, errorDeg, 1.0, "hold error in degrees");
  }

  @Test
  void reachesPresetsAndHoldsAgainstGravity() throws InterruptedException {
    run(0.5);
    DriverStationSim.setEnabled(true);
    run(0.5);
    // Powers on resting on the lower hard stop, like a real arm.
    assertEquals(
        Units.radiansToDegrees(ArmConstants.MIN_ANGLE_RAD),
        Units.radiansToDegrees(arm.getPositionRad()),
        1.0,
        "starting angle");

    // Horizontal is the worst case for gravity, so holding there checks kG.
    goToAndHold(ArmConstants.HORIZONTAL_RAD);
    goToAndHold(ArmConstants.VERTICAL_RAD);
    goToAndHold(ArmConstants.STOWED_RAD);
  }
}
