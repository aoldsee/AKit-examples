package first.robot.subsystems.intake;

import first.robot.util.MotorFaults;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Trigger;
import org.wpilib.math.filter.Debouncer;
import org.wpilib.system.Timer;
import org.wpilib.util.Alert;
import org.wpilib.util.Alert.Level;

/**
 * Rollers that pull a game piece in, hold it, and push it back out, with a beam break that says
 * whether a piece is there.
 *
 * <p>Where the arm finishes when it reaches an angle, these commands finish when the sensor
 * changes: {@link #intake} when a piece arrives, {@link #eject} once it's gone. The robot must call
 * {@link #periodic()} every loop before running the scheduler.
 */
public class Intake implements Mechanism {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private final Alert motorDisconnectedAlert =
      new Alert("Intake", "MotorDisconnected", "Disconnected intake motor.", Level.HIGH);
  private final MotorFaults.Alerts motorFaultAlerts = new MotorFaults.Alerts("Intake", "Intake");

  // The debounce lives here rather than in the IO, so the log keeps the raw sensor reading and
  // replay re-runs this filter like any other robot logic.
  private final Debouncer sensorDebouncer =
      new Debouncer(IntakeConstants.SENSOR_DEBOUNCE_SECONDS, Debouncer.DebounceType.BOTH);
  private boolean hasPiece = false;

  /** True while a piece is in the intake. Bind to it like a button. */
  public final Trigger hasPieceTrigger = new Trigger(this::hasPiece);

  public Intake(IntakeIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
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
