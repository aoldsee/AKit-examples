package first.robot.subsystems.intake;

import first.robot.util.MotorFaults;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.math.filter.Debouncer;
import org.wpilib.system.Timer;
import org.wpilib.util.Alert;
import org.wpilib.util.Alert.Level;

/**
 * Rollers that pull a game piece in, hold it, and push it back out, with a beam break that says
 * whether a piece is there.
 *
 * <p>Its commands finish when the sensor changes: {@link #intake} when a piece arrives, {@link
 * #eject} once it's gone. When either finishes, the default command, {@link #hold} (set in
 * Controls), takes over the rollers again. The robot must call {@link #periodic()} every loop
 * before running the scheduler.
 *
 * <p>{@code run(...)} and {@code runRepeatedly(...)}, used below to build commands, come from the
 * {@link Mechanism} interface.
 */
public class Intake implements Mechanism {
  private final IntakeIO io;
  // Generated at build time from IntakeIO.IntakeIOInputs (see @AutoLog there), so it isn't in
  // src/. AdvantageKit writes the code that logs every field.
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private final Alert motorDisconnectedAlert =
      new Alert("Intake", "MotorDisconnected", "Disconnected intake motor.", Level.HIGH);
  private final MotorFaults.Alerts motorFaultAlerts = new MotorFaults.Alerts("Intake", "Intake");

  // Debouncing ignores a reading until it has held steady for a moment, so a piece bouncing in
  // the rollers can't flicker hasPiece. It lives here rather than in the IO, so the log keeps the
  // raw sensor reading and replay re-runs this filter like any other robot logic.
  private final Debouncer sensorDebouncer =
      new Debouncer(IntakeConstants.SENSOR_DEBOUNCE_SECONDS, Debouncer.DebounceType.BOTH);
  private boolean hasPiece = false;

  public Intake(IntakeIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    // Normally this logs the inputs. In replay it does the opposite: it fills them in from the log,
    // so everything below sees exactly what the robot saw in the match.
    Logger.processInputs("Intake", inputs);
    hasPiece = sensorDebouncer.calculate(inputs.sensorBlocked);
    motorDisconnectedAlert.set(!inputs.motorConnected);
    motorFaultAlerts.update(inputs.motorFaults);
  }

  /** Runs the rollers inward until a piece is in. Finishes right away if one already is. */
  public Command intake() {
    return run(coroutine -> {
          while (!hasPiece) {
            io.setVoltage(IntakeConstants.INTAKE_VOLTS);
            coroutine.yield();
          }
        })
        .named("Intake.Intake");
  }

  /** Pushes the piece out, then keeps going briefly so it's clear of the rollers. */
  public Command eject() {
    return run(coroutine -> {
          while (hasPiece) {
            io.setVoltage(IntakeConstants.EJECT_VOLTS);
            coroutine.yield();
          }
          var timer = Timer.createStarted();
          while (!timer.hasElapsed(IntakeConstants.EJECT_FOLLOW_THROUGH_SECONDS)) {
            io.setVoltage(IntakeConstants.EJECT_VOLTS);
            coroutine.yield();
          }
        })
        .named("Intake.Eject");
  }

  /**
   * The default command: squeeze a held piece gently so it can't slide out, and stop otherwise.
   * Never finishes.
   */
  public Command hold() {
    return runRepeatedly(() -> io.setVoltage(hasPiece ? IntakeConstants.HOLD_VOLTS : 0.0))
        .named("Intake.Hold");
  }

  @AutoLogOutput(key = "Intake/HasPiece")
  public boolean hasPiece() {
    return hasPiece;
  }
}
