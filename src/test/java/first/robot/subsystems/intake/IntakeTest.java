package first.robot.subsystems.intake;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.command3.Scheduler;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.hardware.hal.RobotMode;
import org.wpilib.simulation.DriverStationSim;
import org.wpilib.simulation.SimHooks;

/**
 * Checks Intake's rules with a fake IO: the test decides what the beam break sees and records the
 * voltage Intake asks for. IntakeSimTest runs the real IO against simulated rollers instead.
 */
class IntakeTest {
  private static class FakeIntakeIO implements IntakeIO {
    boolean blocked = false;
    double volts = Double.NaN;

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
      inputs.motorConnected = true;
      inputs.sensorBlocked = blocked;
    }

    @Override
    public void setVoltage(double volts) {
      this.volts = volts;
    }
  }

  private static final FakeIntakeIO io = new FakeIntakeIO();
  private static final Scheduler scheduler = Scheduler.getDefault();
  // One Intake for the whole class: its alerts are registered by name and can't be created twice.
  private static Intake intake;

  @BeforeAll
  static void setup() {
    HAL.initialize();
    // Paused timing, so the test steps the clock itself and the debounce is deterministic.
    SimHooks.pauseTiming();
    intake = new Intake(io);
    intake.setDefaultCommand(intake.hold());
    DriverStationSim.setRobotMode(RobotMode.TELEOPERATED);
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setEnabled(true);
  }

  /** One 20 ms robot loop. */
  private static void loop() {
    SimHooks.stepTiming(0.02);
    DriverStationSim.notifyNewData();
    intake.periodic();
    scheduler.run();
  }

  /** Enough loops for the debounce to settle on the current sensor reading. */
  private static void settle() {
    for (int i = 0; i < 5; i++) {
      loop();
    }
  }

  @BeforeEach
  void startEmpty() {
    scheduler.cancelAll();
    io.blocked = false;
    settle();
  }

  @Test
  void intakeRunsUntilAPieceArrives() {
    var command = intake.intake();
    scheduler.schedule(command);
    loop();
    assertEquals(IntakeConstants.INTAKE_VOLTS, io.volts);

    io.blocked = true;
    settle();
    assertTrue(intake.hasPiece());
    assertFalse(scheduler.isScheduledOrRunning(command), "finished once the piece was in");
    // The default command takes over and squeezes the piece.
    loop();
    assertEquals(IntakeConstants.HOLD_VOLTS, io.volts);
  }

  @Test
  void aOneLoopFlickerIsIgnored() {
    var command = intake.intake();
    scheduler.schedule(command);
    io.blocked = true;
    loop();
    io.blocked = false;
    settle();

    assertFalse(intake.hasPiece());
    assertTrue(scheduler.isScheduledOrRunning(command), "a flicker isn't a piece");
  }

  @Test
  void ejectFollowsThroughAfterThePieceIsGone() {
    io.blocked = true;
    settle();
    var command = intake.eject();
    scheduler.schedule(command);
    loop();
    assertEquals(IntakeConstants.EJECT_VOLTS, io.volts);

    io.blocked = false;
    settle();
    // Still pushing for the follow-through time after the sensor cleared.
    assertTrue(scheduler.isScheduledOrRunning(command));
    for (int i = 0; i < 15; i++) {
      loop();
    }
    assertFalse(scheduler.isScheduledOrRunning(command));
    loop();
    assertEquals(0.0, io.volts, "empty intake rests");
  }
}
