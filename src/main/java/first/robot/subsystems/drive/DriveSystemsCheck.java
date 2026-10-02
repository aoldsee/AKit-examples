package first.robot.subsystems.drive;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Command;
import org.wpilib.command3.Coroutine;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.system.Timer;
import org.wpilib.util.Alert;
import org.wpilib.util.Alert.Level;

/**
 * A quick pit check that every swerve module steers and drives. Run it from the auto chooser with
 * the robot on blocks (or with a couple of meters of clear floor ahead), and read the result from
 * the alerts.
 *
 * <p>It steers every module to 90 degrees and back to 0, checking each one gets there, then runs
 * every drive motor forward on the same voltage and checks each wheel spins forward at about the
 * same speed as the others. That catches a dead or unplugged motor, a steer motor that can't reach
 * its angle, a drive motor spinning backward (a wrong invert), and a module that's binding.
 *
 * <p>What it can't catch: a wrong CANcoder offset. The module steers to where its encoder says 0
 * is, so the check passes even if that's crooked. At the end every wheel should be pointing
 * straight ahead; look.
 */
public final class DriveSystemsCheck {
  private DriveSystemsCheck() {}

  private static final double[] STEER_TEST_ANGLES_DEG = {90.0, 0.0};
  private static final double STEER_TOLERANCE_DEG = 5.0;
  private static final double STEER_SETTLE_SECONDS = 0.75;

  // Volts. About a sixth of full speed: enough to see, slow enough to be safe on the floor.
  private static final double DRIVE_TEST_VOLTS = 2.0;
  private static final double DRIVE_SPINUP_SECONDS = 1.0;
  // m/s. At 2 V a healthy wheel turns several times faster than this.
  private static final double MIN_WHEEL_SPEED = 0.2;
  // A wheel this far from the other wheels' average speed (as a fraction of it) is binding.
  private static final double MAX_SPEED_MISMATCH = 0.25;

  // One set of alerts, shared by every run, because alerts can only be created once.
  private static final Alert passedAlert =
      new Alert("SystemsCheck", "DrivePassed", "Drive systems check passed.", Level.LOW);
  private static final Alert failedAlert = new Alert("SystemsCheck", "DriveFailed", "", Level.HIGH);

  public static Command create(Drive drive) {
    return create(drive, failures -> {});
  }

  /** As above, and hands the list of failures (empty if it passed) to {@code onFinish}. */
  public static Command create(Drive drive, Consumer<List<String>> onFinish) {
    return drive
        .run(
            coroutine -> {
              passedAlert.set(false);
              failedAlert.set(false);
              List<String> failures = new ArrayList<>();

              for (double angleDeg : STEER_TEST_ANGLES_DEG) {
                var angle = Rotation2d.fromDegrees(angleDeg);
                hold(coroutine, drive, angle, 0.0, STEER_SETTLE_SECONDS);
                var modules = drive.getModuleVelocities();
                for (int i = 0; i < modules.length; i++) {
                  double errorDeg = angle.minus(modules[i].angle).getDegrees();
                  if (Math.abs(errorDeg) > STEER_TOLERANCE_DEG) {
                    failures.add(
                        String.format(
                            "Module %d steered to %.0f deg instead of %.0f.",
                            i, modules[i].angle.getDegrees(), angleDeg));
                  }
                }
              }

              hold(coroutine, drive, Rotation2d.ZERO, DRIVE_TEST_VOLTS, DRIVE_SPINUP_SECONDS);
              var modules = drive.getModuleVelocities();
              // First the wheels that clearly failed: backward or barely turning.
              List<Integer> spinning = new ArrayList<>();
              for (int i = 0; i < modules.length; i++) {
                double speed = modules[i].velocity;
                if (speed < -MIN_WHEEL_SPEED) {
                  failures.add(
                      String.format("Module %d drove backward. Check its drive invert.", i));
                } else if (speed < MIN_WHEEL_SPEED) {
                  failures.add(
                      String.format("Module %d barely drove (%.2f m/s). Dead motor?", i, speed));
                } else {
                  spinning.add(i);
                }
              }
              // Then compare each remaining wheel with the other remaining ones. Leaving the
              // failed wheels out of the average matters: a dead wheel would drag it down and
              // make every healthy wheel look too fast.
              for (int i : spinning) {
                double others = 0.0;
                for (int j : spinning) {
                  if (j != i) {
                    others += modules[j].velocity / (spinning.size() - 1);
                  }
                }
                double speed = modules[i].velocity;
                if (spinning.size() > 1 && Math.abs(speed - others) > MAX_SPEED_MISMATCH * others) {
                  failures.add(
                      String.format(
                          "Module %d drove at %.2f m/s, the others average %.2f. Binding?",
                          i, speed, others));
                }
              }
              drive.runDirect(Rotation2d.ZERO, 0.0);

              report(failures);
              onFinish.accept(failures);
            })
        .named("Drive.SystemsCheck");
  }

  /** Holds every module at one angle and drive output for a while. */
  private static void hold(
      Coroutine coroutine, Drive drive, Rotation2d angle, double driveVolts, double seconds) {
    var timer = Timer.createStarted();
    while (!timer.hasElapsed(seconds)) {
      drive.runDirect(angle, driveVolts);
      coroutine.yield();
    }
  }

  private static void report(List<String> failures) {
    Logger.recordOutput("SystemsCheck/Drive/Failures", failures.toArray(new String[0]));
    if (failures.isEmpty()) {
      passedAlert.set(true);
      System.out.println("Drive systems check passed. Check every wheel points straight ahead.");
    } else {
      failedAlert.setText("Drive systems check failed: " + String.join(" ", failures));
      failedAlert.set(true);
      System.out.println("Drive systems check failed:");
      failures.forEach(failure -> System.out.println("  " + failure));
    }
  }
}
