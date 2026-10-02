package first.robot.subsystems.drive;

import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.DoubleUnaryOperator;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Command;
import org.wpilib.driverstation.RobotState;
import org.wpilib.math.filter.SlewRateLimiter;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.util.MathUtil;
import org.wpilib.math.util.Units;
import org.wpilib.sysid.SysIdRoutineLog;
import org.wpilib.system.Timer;

/**
 * Routines that measure the drivetrain instead of driving it somewhere: feedforward and wheel
 * radius characterization, and SysId tests. Each runs from the auto chooser.
 */
public final class DriveCharacterization {
  // Seconds to hold zero output so modules can finish turning before data collection.
  private static final double FF_START_DELAY = 2.0;

  // Volts per second. Slow enough that acceleration is negligible and kA can be ignored.
  private static final double FF_RAMP_RATE = 0.1;

  // Wheel rotations/s. Samples below this are static friction, not the linear region, and would
  // drag the fit toward a lower kS and higher kV.
  private static final double FF_MIN_VELOCITY = 0.05;

  // Rad/s and rad/s^2. Slow spin keeps wheels from slipping, which would inflate the radius.
  private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25;

  private static final double WHEEL_RADIUS_RAMP_RATE = 0.05;

  // SysId test parameters, matching WPILib's SysIdRoutine defaults. Volts/s, volts, seconds.
  private static final double SYSID_RAMP_RATE = 1.0;
  private static final double SYSID_STEP_VOLTAGE = 7.0;
  private static final double SYSID_TIMEOUT = 10.0;
  // Seconds at zero output before each test so the modules finish turning straight.
  private static final double SYSID_SETTLE_TIME = 1.0;

  private static final String SYSID_STATE_KEY = "Drive/SysIdState";

  public enum SysIdDirection {
    FORWARD,
    REVERSE
  }

  private DriveCharacterization() {}

  /**
   * Ramps drive voltage and fits kS and kV for the drive motors. Runs until canceled, then prints
   * the fit. Only meaningful with Voltage closed-loop output.
   */
  public static Command feedforwardCharacterization(Drive drive) {
    return feedforwardCharacterization(drive, DriveCharacterization::printFeedforwardFit);
  }

  /** As above, but hands the fit to {@code onResult} instead of printing it. */
  public static Command feedforwardCharacterization(
      Drive drive, Consumer<FeedforwardFit> onResult) {
    List<Double> velocitySamples = new ArrayList<>();
    List<Double> voltageSamples = new ArrayList<>();

    return drive
        .run(
            coroutine -> {
              velocitySamples.clear();
              voltageSamples.clear();

              var timer = Timer.createStarted();
              while (!timer.hasElapsed(FF_START_DELAY)) {
                drive.runCharacterization(0.0);
                coroutine.yield();
              }

              timer.restart();
              while (true) {
                double voltage = timer.get() * FF_RAMP_RATE;
                drive.runCharacterization(voltage);
                double velocity = drive.getFFCharacterizationVelocity();
                if (Math.abs(velocity) > FF_MIN_VELOCITY) {
                  velocitySamples.add(velocity);
                  voltageSamples.add(voltage);
                }
                coroutine.yield();
              }
            })
        .whenCanceled(
            () -> {
              if (velocitySamples.size() < 2) {
                System.out.println("Drive FF characterization canceled before collecting data.");
                return;
              }
              onResult.accept(fitFeedforward(velocitySamples, voltageSamples));
            })
        .named("Drive.FeedforwardCharacterization");
  }

  /**
   * Spins in place and compares gyro rotation with wheel rotation to measure the effective wheel
   * radius. Runs until canceled, then prints the result. Needs at least one full rotation.
   */
  public static Command wheelRadiusCharacterization(Drive drive) {
    return wheelRadiusCharacterization(drive, DriveCharacterization::printWheelRadius);
  }

  /** As above, but hands the result to {@code onResult} instead of printing it. */
  public static Command wheelRadiusCharacterization(
      Drive drive, Consumer<WheelRadiusResult> onResult) {
    var limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
    var state = new WheelRadiusState();

    return drive
        .run(
            coroutine -> {
              limiter.reset(0.0);
              state.started = false;

              var timer = Timer.createStarted();
              while (true) {
                double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                drive.runVelocity(new ChassisVelocities(0.0, 0.0, speed));

                // The first second lets the modules finish turning to the spin orientation.
                if (!state.started && timer.hasElapsed(1.0)) {
                  state.started = true;
                  state.startPositions = drive.getWheelRadiusCharacterizationPositions();
                  state.lastAngle = drive.getRotation();
                  state.gyroDelta = 0.0;
                } else if (state.started) {
                  var rotation = drive.getRotation();
                  state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                  state.lastAngle = rotation;
                }
                coroutine.yield();
              }
            })
        .whenCanceled(
            () -> {
              if (!state.started) {
                System.out.println(
                    "Wheel radius characterization canceled before collecting data.");
                return;
              }
              onResult.accept(measureWheelRadius(drive, state));
            })
        .named("Drive.WheelRadiusCharacterization");
  }

  /**
   * SysId quasistatic test: drive voltage ramps slowly, so velocity tracks voltage and kS/kV can be
   * fit. Load the AdvantageKit log into the SysId tool and pick {@code Drive/SysIdState} as the
   * test state.
   */
  public static Command sysIdQuasistatic(Drive drive, SysIdDirection direction) {
    boolean forward = direction == SysIdDirection.FORWARD;
    return sysIdTest(
        drive,
        forward
            ? SysIdRoutineLog.State.QUASISTATIC_FORWARD
            : SysIdRoutineLog.State.QUASISTATIC_REVERSE,
        t -> (forward ? 1 : -1) * SYSID_RAMP_RATE * t);
  }

  /** SysId dynamic test: a voltage step, so the acceleration response gives kA. */
  public static Command sysIdDynamic(Drive drive, SysIdDirection direction) {
    boolean forward = direction == SysIdDirection.FORWARD;
    return sysIdTest(
        drive,
        forward ? SysIdRoutineLog.State.DYNAMIC_FORWARD : SysIdRoutineLog.State.DYNAMIC_REVERSE,
        t -> (forward ? 1 : -1) * SYSID_STEP_VOLTAGE);
  }

  /**
   * Shared body of the SysId tests. WPILib's SysIdRoutine is built on Commands v2, so this does the
   * same thing as a v3 coroutine. AdvantageKit already logs the voltage, position, and velocity
   * inputs SysId needs; only the test state is recorded here.
   */
  private static Command sysIdTest(
      Drive drive, SysIdRoutineLog.State state, DoubleUnaryOperator voltageAtTime) {
    return drive
        .run(
            coroutine -> {
              var timer = Timer.createStarted();
              while (!timer.hasElapsed(SYSID_SETTLE_TIME)) {
                drive.runCharacterization(0.0);
                Logger.recordOutput(SYSID_STATE_KEY, SysIdRoutineLog.State.NONE.toString());
                coroutine.yield();
              }

              timer.restart();
              while (!timer.hasElapsed(SYSID_TIMEOUT)) {
                drive.runCharacterization(voltageAtTime.applyAsDouble(timer.get()));
                Logger.recordOutput(SYSID_STATE_KEY, state.toString());
                coroutine.yield();
              }

              drive.runCharacterization(0.0);
              Logger.recordOutput(SYSID_STATE_KEY, SysIdRoutineLog.State.NONE.toString());
            })
        // Without this the log would show the test still running after a cancel.
        .whenCanceled(
            () -> Logger.recordOutput(SYSID_STATE_KEY, SysIdRoutineLog.State.NONE.toString()))
        .named("Drive.SysId[" + state + "]");
  }

  /**
   * Works out each module's CANcoder offset. Run it with the robot disabled and every wheel lined
   * up by hand to point straight ahead, with all the bevel gears facing the same side (a
   * straightedge along each side of the robot helps). It prints the new offsets ready to paste into
   * {@link DriveConstants#MODULES}, and logs them under {@code Calibration/EncoderOffsets}.
   *
   * <p>If a wheel ends up driving backward afterward (the systems check catches this), that module
   * was lined up 180 degrees off: turn it around and run this again.
   */
  public static Command findEncoderOffsets(Drive drive) {
    return Command.noRequirements(
            coroutine -> {
              if (!RobotState.isDisabled()) {
                System.out.println("Encoder offsets not measured: disable the robot first.");
                return;
              }
              double[] offsets = encoderOffsets(drive.getModuleAbsoluteAngles());
              Logger.recordOutput("Calibration/EncoderOffsets", offsets);
              var formatter = new DecimalFormat("0.00000");
              System.out.println("********** Encoder Offsets **********");
              System.out.println("Rotations, in DriveConstants.MODULES order:");
              String[] names = {"Front left", "Front right", "Back left", "Back right"};
              for (int i = 0; i < offsets.length; i++) {
                System.out.println("\t" + names[i] + ": " + formatter.format(offsets[i]));
              }
            })
        .named("Drive.FindEncoderOffsets");
  }

  /**
   * The offsets that would make each module read 0 right now. The CANcoder adds its configured
   * offset to the raw magnet angle, so the reading is raw + current offset. To read 0 instead, the
   * new offset is current offset - reading. Wrapped to [-0.5, 0.5) rotations, the range the
   * CANcoder accepts. Package-private for the unit tests.
   */
  static double[] encoderOffsets(Rotation2d[] readings) {
    double[] offsets = new double[readings.length];
    for (int i = 0; i < readings.length; i++) {
      double current = DriveConstants.MODULES[i].encoderOffsetRotations();
      offsets[i] = MathUtil.inputModulus(current - readings[i].getRotations(), -0.5, 0.5);
    }
    return offsets;
  }

  /** kV is volts per wheel rotation/s, matching DriveConstants.DRIVE_GAINS. */
  public record FeedforwardFit(double kS, double kV) {}

  /** Wheel and gyro deltas in radians, radius in meters. */
  public record WheelRadiusResult(double wheelDeltaRad, double gyroDeltaRad, double radiusMeters) {}

  /**
   * Least-squares fit of voltage = kS + kV * velocity: the same line of best fit a spreadsheet
   * trendline draws through the points. kS is where the line crosses zero velocity (the voltage
   * needed just to overcome friction), and kV is its slope.
   */
  private static FeedforwardFit fitFeedforward(List<Double> velocities, List<Double> voltages) {
    int n = velocities.size();
    double sumX = 0.0;
    double sumY = 0.0;
    double sumXY = 0.0;
    double sumX2 = 0.0;
    for (int i = 0; i < n; i++) {
      double x = velocities.get(i);
      double y = voltages.get(i);
      sumX += x;
      sumY += y;
      sumXY += x * y;
      sumX2 += x * x;
    }
    double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
    double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);
    return new FeedforwardFit(kS, kV);
  }

  private static WheelRadiusResult measureWheelRadius(Drive drive, WheelRadiusState state) {
    double[] positions = drive.getWheelRadiusCharacterizationPositions();
    double wheelDelta = 0.0;
    for (int i = 0; i < positions.length; i++) {
      wheelDelta += Math.abs(positions[i] - state.startPositions[i]) / positions.length;
    }
    // Each wheel travels the arc the robot turns through at the drive base radius.
    double radius = (state.gyroDelta * DriveConstants.DRIVE_BASE_RADIUS) / wheelDelta;
    return new WheelRadiusResult(wheelDelta, state.gyroDelta, radius);
  }

  private static void printFeedforwardFit(FeedforwardFit fit) {
    var formatter = new DecimalFormat("#0.00000");
    System.out.println("********** Drive FF Characterization Results **********");
    System.out.println("\tkS: " + formatter.format(fit.kS()));
    System.out.println("\tkV: " + formatter.format(fit.kV()) + " (V per wheel rotation/s)");
  }

  private static void printWheelRadius(WheelRadiusResult result) {
    var formatter = new DecimalFormat("#0.000");
    System.out.println("********** Wheel Radius Characterization Results **********");
    System.out.println("\tWheel Delta: " + formatter.format(result.wheelDeltaRad()) + " radians");
    System.out.println("\tGyro Delta: " + formatter.format(result.gyroDeltaRad()) + " radians");
    System.out.println(
        "\tWheel Radius: "
            + formatter.format(result.radiusMeters())
            + " meters, "
            + formatter.format(Units.metersToInches(result.radiusMeters()))
            + " inches");
  }

  private static final class WheelRadiusState {
    boolean started = false;
    double[] startPositions = new double[4];
    Rotation2d lastAngle = Rotation2d.ZERO;
    double gyroDelta = 0.0;
  }
}
