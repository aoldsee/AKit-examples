package first.robot.subsystems.arm;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.command3.Scheduler;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.hardware.hal.RobotMode;
import org.wpilib.math.util.Units;
import org.wpilib.simulation.DriverStationSim;

/**
 * Checks Arm's rules with a fake IO: the test decides where the arm "is" and records what Arm tells
 * the motor. Compare with ArmSimTest, which runs real physics in real time to check the arm
 * actually gets there.
 */
class ArmTest {
  /** Reports a position the test chooses and remembers the last thing it was told to do. */
  private static class FakeArmIO implements ArmIO {
    double positionRad = 0.0;
    Double commandedPositionRad = null;
    Double commandedVolts = null;
    Gains sentGains = null;

    @Override
    public void updateInputs(ArmIOInputs inputs) {
      inputs.motorConnected = true;
      inputs.encoderConnected = true;
      inputs.positionRad = positionRad;
      inputs.absolutePositionRad = positionRad;
    }

    @Override
    public void setPosition(double angleRad) {
      commandedPositionRad = angleRad;
    }

    @Override
    public void setVoltage(double volts) {
      commandedVolts = volts;
    }

    @Override
    public void setGains(Gains gains) {
      sentGains = gains;
    }

    void forgetCommands() {
      commandedPositionRad = null;
      commandedVolts = null;
    }
  }

  private static final FakeArmIO io = new FakeArmIO();
  private static final Scheduler scheduler = Scheduler.getDefault();
  // One Arm for the whole class: its alerts are registered by name and can't be created twice.
  private static Arm arm;

  @BeforeAll
  static void setup() {
    HAL.initialize();
    arm = new Arm(io);
    arm.setDefaultCommand(arm.hold());
    DriverStationSim.setRobotMode(RobotMode.TELEOPERATED);
    DriverStationSim.setDsAttached(true);
  }

  /** One robot loop: read inputs, then run commands. */
  private static void loop() {
    DriverStationSim.notifyNewData();
    arm.periodic();
    scheduler.run();
  }

  private static void setEnabled(boolean enabled) {
    DriverStationSim.setEnabled(enabled);
    loop();
  }

  @BeforeEach
  void startDisabled() {
    scheduler.cancelAll();
    setEnabled(false);
    io.forgetCommands();
  }

  @Test
  void enablingOnTheHardStopDoesNotLiftTheArm() {
    io.positionRad = ArmConstants.MIN_ANGLE_RAD;
    loop();
    setEnabled(true);
    loop();

    // Outside the soft limits, holding means letting it rest, not driving to the soft limit.
    assertNull(io.commandedPositionRad);
    assertEquals(0.0, io.commandedVolts);
  }

  @Test
  void enablingHoldsWhereTheArmIsNow() {
    // Someone moved the arm by hand while disabled.
    io.positionRad = Units.degreesToRadians(10.0);
    loop();
    setEnabled(true);
    loop();

    assertEquals(Units.degreesToRadians(10.0), io.commandedPositionRad, 1e-9);
  }

  @Test
  void goToFinishesOnArrivalAndKeepsHolding() {
    io.positionRad = 0.0;
    loop();
    setEnabled(true);

    var command = arm.goTo(ArmConstants.VERTICAL_RAD);
    scheduler.schedule(command);
    loop();
    assertEquals(ArmConstants.VERTICAL_RAD, io.commandedPositionRad, 1e-9);
    assertTrue(scheduler.isScheduledOrRunning(command), "still traveling");

    io.positionRad = ArmConstants.VERTICAL_RAD;
    loop();
    loop();
    assertFalse(scheduler.isScheduledOrRunning(command), "finished once there");

    // The default hold command takes over and keeps the same goal.
    io.forgetCommands();
    loop();
    assertEquals(ArmConstants.VERTICAL_RAD, io.commandedPositionRad, 1e-9);
  }

  @Test
  void goalsPastTheSoftLimitsAreClamped() {
    setEnabled(true);
    scheduler.schedule(arm.goTo(Units.degreesToRadians(200.0)));
    loop();

    assertEquals(ArmConstants.SOFT_MAX_ANGLE_RAD, io.commandedPositionRad, 1e-9);
  }

  @Test
  void unchangedGainsAreNotResent() {
    loop();
    loop();
    // Re-sending gains every loop would block on the CAN bus each time.
    assertNull(io.sentGains);
  }

  @Test
  void nearHardStopOnlyAtTheEndsOfTravel() {
    io.positionRad = ArmConstants.MIN_ANGLE_RAD;
    loop();
    assertTrue(arm.isNearHardStop(), "resting on the lower stop");

    io.positionRad = ArmConstants.STOWED_RAD;
    loop();
    assertTrue(arm.isNearHardStop(), "the stowed preset counts");

    io.positionRad = ArmConstants.HORIZONTAL_RAD;
    loop();
    assertFalse(arm.isNearHardStop(), "held out by the motor");

    io.positionRad = ArmConstants.MAX_ANGLE_RAD;
    loop();
    assertTrue(arm.isNearHardStop(), "resting on the upper stop");
  }
}
