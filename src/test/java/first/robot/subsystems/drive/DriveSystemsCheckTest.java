package first.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.command3.Scheduler;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.hardware.hal.RobotMode;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.simulation.DriverStationSim;
import org.wpilib.simulation.SimHooks;

/**
 * Runs the systems check against fake modules that can be broken on purpose, to show it notices
 * each kind of problem. No physics: a fake module steers instantly and spins at a speed
 * proportional to its voltage.
 */
class DriveSystemsCheckTest {
  private static class FakeModuleIO implements ModuleIO {
    boolean steerStuck = false;
    boolean driveInverted = false;
    boolean driveDead = false;
    double slowFactor = 1.0;
    Rotation2d angle = Rotation2d.ZERO;
    double driveVolts = 0.0;

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
      inputs.driveConnected = true;
      inputs.turnConnected = true;
      inputs.turnEncoderConnected = true;
      inputs.turnPosition = angle;
      // Wheel rad/s: roughly 30 per volt, like an unloaded MK5n wheel.
      double radPerSec = driveDead ? 0.0 : 30.0 * driveVolts * slowFactor;
      inputs.driveVelocityRadPerSec = driveInverted ? -radPerSec : radPerSec;
    }

    @Override
    public void setDriveOpenLoop(double output) {
      driveVolts = output;
    }

    @Override
    public void setTurnPosition(Rotation2d rotation) {
      if (!steerStuck) {
        angle = rotation;
      }
    }

    @Override
    public void setTurnOpenLoop(double output) {}
  }

  private static final FakeModuleIO[] modules = {
    new FakeModuleIO(), new FakeModuleIO(), new FakeModuleIO(), new FakeModuleIO()
  };
  private static final Scheduler scheduler = Scheduler.getDefault();
  // One Drive for the whole class: its alerts are registered by name and can't be created twice.
  private static Drive drive;

  @BeforeAll
  static void setup() {
    HAL.initialize();
    // Paused timing, so the check's waits are stepped through instantly.
    SimHooks.pauseTiming();
    drive = new Drive(new GyroIO() {}, modules[0], modules[1], modules[2], modules[3]);
    DriverStationSim.setRobotMode(RobotMode.AUTONOMOUS);
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setEnabled(true);
  }

  @BeforeEach
  void healthyModules() {
    for (var module : modules) {
      module.steerStuck = false;
      module.driveInverted = false;
      module.driveDead = false;
      module.slowFactor = 1.0;
      module.angle = Rotation2d.ZERO;
    }
  }

  private static List<String> runCheck() {
    List<String> result = new ArrayList<>();
    var command = DriveSystemsCheck.create(drive, result::addAll);
    scheduler.schedule(command);
    // The check takes about 2.5 s; run 3 s of 20 ms loops.
    for (int i = 0; i < 150; i++) {
      SimHooks.stepTiming(0.02);
      DriverStationSim.notifyNewData();
      drive.periodic();
      scheduler.run();
    }
    assertTrue(!scheduler.isScheduledOrRunning(command), "the check should finish on its own");
    return result;
  }

  @Test
  void healthyDrivetrainPasses() {
    assertEquals(List.of(), runCheck());
  }

  @Test
  void aSlowWheelIsCalledOut() {
    // Spinning at well under the others' speed, as a binding module would.
    modules[2].slowFactor = 0.5;
    assertEquals(
        List.of("Module 2 drove at 1.52 m/s, the others average 3.05. Binding?"), runCheck());
  }

  @Test
  void eachProblemIsReportedAgainstTheRightModule() {
    modules[1].steerStuck = true;
    modules[2].driveInverted = true;
    modules[3].driveDead = true;

    var failures = runCheck();

    // Module 1 never left 0, so only the 90 degree step fails.
    assertEquals(
        List.of(
            "Module 1 steered to 0 deg instead of 90.",
            "Module 2 drove backward. Check its drive invert.",
            "Module 3 barely drove (0.00 m/s). Dead motor?"),
        failures);
  }
}
