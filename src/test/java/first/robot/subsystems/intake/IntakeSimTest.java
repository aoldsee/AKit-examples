package first.robot.subsystems.intake;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.ctre.phoenix6.SignalLogger;
import first.robot.sim.SimWorld;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import org.wpilib.command3.Scheduler;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.hardware.hal.RobotMode;
import org.wpilib.simulation.DriverStationSim;

/** Runs the intake's real IO against IntakeSim in real time: eject the preload, then grab one. */
@Tag("sim")
class IntakeSimTest {
  private static SimWorld simWorld;
  private static IntakeSim intakeSim;
  private static Intake intake;
  private static final Scheduler scheduler = Scheduler.getDefault();

  @BeforeAll
  static void setup() {
    HAL.initialize();
    SignalLogger.enableAutoLogging(false);
    simWorld = new SimWorld();
    intakeSim = new IntakeSim(true);
    simWorld.add("Intake", intakeSim);
    intake = new Intake(new IntakeIOTalonFXSim(intakeSim));
    intake.setDefaultCommand(intake.hold());
    simWorld.start();
    DriverStationSim.setRobotMode(RobotMode.TELEOPERATED);
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setEnabled(true);
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
      intake.periodic();
      scheduler.run();
      Thread.sleep(20);
    }
  }

  @Test
  void ejectsThePreloadThenIntakesAnother() throws InterruptedException {
    run(0.5);
    assertTrue(intake.hasPiece(), "starts with the preload");

    var eject = intake.eject();
    scheduler.schedule(eject);
    run(1.5);
    assertFalse(scheduler.isScheduledOrRunning(eject), "eject should finish on its own");
    assertFalse(intakeSim.hasPiece());
    assertFalse(intake.hasPiece());

    var grab = intake.intake();
    scheduler.schedule(grab);
    run(1.5);
    assertFalse(scheduler.isScheduledOrRunning(grab), "intake should finish once the piece is in");
    assertTrue(intake.hasPiece());

    // Holding: the rollers are stalled against the piece, not spinning.
    run(0.5);
    assertTrue(intake.hasPiece(), "still holding");
  }
}
