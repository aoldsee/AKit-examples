package first.robot.subsystems.arm;

import first.robot.util.MotorFaults;
import first.robot.util.TunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Trigger;
import org.wpilib.driverstation.RobotState;
import org.wpilib.math.util.Units;
import org.wpilib.util.Alert;
import org.wpilib.util.Alert.Level;
import org.wpilib.util.Color8Bit;

/**
 * A single-jointed arm that moves between angles and holds them against gravity.
 *
 * <p>Commands own the motor output: {@link #goTo} moves to a goal and finishes on arrival, and
 * {@link #hold} (the default command) keeps commanding the last goal. Like {@link
 * first.robot.subsystems.drive.Drive}, the robot must call {@link #periodic()} every loop before
 * running the scheduler.
 */
public class Arm implements Mechanism {
  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
  private final Alert motorDisconnectedAlert =
      new Alert("Arm", "MotorDisconnected", "Disconnected arm motor.", Level.HIGH);
  private final Alert encoderDisconnectedAlert =
      new Alert("Arm", "EncoderDisconnected", "Disconnected arm encoder.", Level.HIGH);
  private final MotorFaults.Alerts motorFaultAlerts = new MotorFaults.Alerts("Arm", "Arm");

  // NaN until set while disabled (see periodic), so hold() commands nothing before then.
  private double goalRad = Double.NaN;

  // Measured arm and goal arm drawn side by side. Origin is the pivot, sized to fit the arm.
  private final LoggedMechanism2d mechanism2d = new LoggedMechanism2d(1.2, 1.2);
  private final LoggedMechanismLigament2d measuredLigament =
      mechanism2d
          .getRoot("Pivot", 0.6, 0.6)
          .append(
              new LoggedMechanismLigament2d(
                  "Measured", ArmConstants.LENGTH_METERS, 0.0, 6, new Color8Bit(255, 165, 0)));
  private final LoggedMechanismLigament2d goalLigament =
      mechanism2d
          .getRoot("GoalPivot", 0.6, 0.6)
          .append(
              new LoggedMechanismLigament2d(
                  "Goal", ArmConstants.LENGTH_METERS, 0.0, 2, new Color8Bit(0, 200, 255)));

  // Editable from the dashboard under /Tuning/Arm/ in tuning mode. Try zeroing kG and watch the
  // arm sag, or raising kP until it overshoots.
  private final TunableNumber kP = new TunableNumber("Arm/kP", ArmConstants.GAINS.kP());
  private final TunableNumber kD = new TunableNumber("Arm/kD", ArmConstants.GAINS.kD());
  private final TunableNumber kG = new TunableNumber("Arm/kG", ArmConstants.GAINS.kG());
  private final TunableNumber cruiseVelocity =
      new TunableNumber("Arm/CruiseVelocity", ArmConstants.GAINS.cruiseVelocity());
  private final TunableNumber acceleration =
      new TunableNumber("Arm/Acceleration", ArmConstants.GAINS.acceleration());
  // What the motor controller currently has, so gains are only re-sent when one changes.
  private ArmIO.Gains appliedGains = ArmConstants.GAINS;

  /** True while the arm is within tolerance of its goal. Bind to it like a button. */
  public final Trigger atGoal = new Trigger(this::isAtGoal);

  /** True while the arm is at or near either hard stop. See {@link #isNearHardStop}. */
  public final Trigger nearHardStop = new Trigger(this::isNearHardStop);

  public Arm(ArmIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);

    var gains =
        new ArmIO.Gains(kP.get(), kD.get(), kG.get(), cruiseVelocity.get(), acceleration.get());
    if (!gains.equals(appliedGains)) {
      io.setGains(gains);
      appliedGains = gains;
    }

    // While disabled the arm can be moved by hand or sag. Following it means enabling holds where
    // it is now instead of snapping back to an old goal. Robot code always starts disabled, so
    // this also sets the first goal. Seeding it at construction instead would use the 0 the
    // sensors report before their first CAN frame, and swing the arm to horizontal on enable.
    if (RobotState.isDisabled()) {
      goalRad = inputs.positionRad;
    }

    measuredLigament.setAngle(Units.radiansToDegrees(inputs.positionRad));
    goalLigament.setAngle(Units.radiansToDegrees(goalRad));
    Logger.recordOutput("Arm/Mechanism2d", mechanism2d);

    motorDisconnectedAlert.set(!inputs.motorConnected);
    encoderDisconnectedAlert.set(!inputs.encoderConnected);
    motorFaultAlerts.update(inputs.motorFaults);
  }

  /**
   * Moves to {@code angleRad} and finishes once there. The arm keeps holding the angle afterward
   * through the default {@link #hold} command.
   */
  public Command goTo(double angleRad) {
    return run(coroutine -> {
          goalRad =
              Math.clamp(
                  angleRad, ArmConstants.SOFT_MIN_ANGLE_RAD, ArmConstants.SOFT_MAX_ANGLE_RAD);
          while (!isAtGoal()) {
            io.setPosition(goalRad);
            coroutine.yield();
          }
        })
        .named("Arm.GoTo[" + Math.round(Units.radiansToDegrees(angleRad)) + " deg]");
  }

  /** Keeps commanding the current goal. Never finishes; meant as the default command. */
  public Command hold() {
    return runRepeatedly(
            () -> {
              if (Double.isNaN(goalRad)) {
                return;
              }
              if (goalRad < ArmConstants.SOFT_MIN_ANGLE_RAD
                  || goalRad > ArmConstants.SOFT_MAX_ANGLE_RAD) {
                // Outside the soft limits means the arm is resting on a hard stop, which gravity
                // presses it into at both ends. Holding position there would make the Talon drive
                // it to the soft limit, so the arm would lift every time the robot enabled.
                io.setVoltage(0.0);
              } else {
                io.setPosition(goalRad);
              }
            })
        .named("Arm.Hold");
  }

  @AutoLogOutput(key = "Arm/GoalRad")
  public double getGoalRad() {
    return goalRad;
  }

  public double getPositionRad() {
    return inputs.positionRad;
  }

  /**
   * At or near either end of travel. Near a stop, the stop takes the arm's weight, and it can't
   * swing far before hitting it. Anywhere in between, only the motor holds it up, so other parts of
   * the robot use this to go easier on it (see RobotContainer).
   */
  @AutoLogOutput(key = "Arm/NearHardStop")
  public boolean isNearHardStop() {
    return inputs.positionRad < ArmConstants.MIN_ANGLE_RAD + ArmConstants.NEAR_STOP_MARGIN_RAD
        || inputs.positionRad > ArmConstants.MAX_ANGLE_RAD - ArmConstants.NEAR_STOP_MARGIN_RAD;
  }

  @AutoLogOutput(key = "Arm/AtGoal")
  public boolean isAtGoal() {
    return Math.abs(inputs.positionRad - goalRad) < ArmConstants.AT_GOAL_TOLERANCE_RAD;
  }
}
