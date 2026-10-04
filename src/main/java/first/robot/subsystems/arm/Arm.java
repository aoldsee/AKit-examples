package first.robot.subsystems.arm;

import first.robot.util.MotorFaults;
import first.robot.util.TunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.driverstation.RobotState;
import org.wpilib.math.util.Units;
import org.wpilib.util.Alert;
import org.wpilib.util.Alert.Level;
import org.wpilib.util.Color8Bit;

/**
 * A single-jointed arm that moves between angles and holds them against gravity.
 *
 * <p>Commands own the motor output: {@link #goTo} moves to a goal and finishes on arrival, and
 * {@link #hold} (the default command, set in Controls) keeps commanding the last goal. Like {@link
 * first.robot.subsystems.drive.Drive}, the robot must call {@link #periodic()} every loop before
 * running the scheduler.
 *
 * <p>{@code run(...)} and {@code runRepeatedly(...)}, used below to build commands, come from the
 * {@link Mechanism} interface.
 *
 * <p>The arm's PID doesn't run here. This class decides the goal angle; {@code io.setPosition}
 * hands it to the motor controller, which runs the control loop itself, a thousand times a second.
 * See ArmIOTalonFX for its setup and gains.
 *
 * <p>Angles are in radians, like most of WPILib's math (a full turn is 2 pi, so 90 degrees is about
 * 1.57). The arm has two kinds of limit: hard stops are the physical metal it rests against, and
 * soft limits are a few degrees inside them, where the code stops so the motor never drives into
 * the metal.
 */
public class Arm implements Mechanism {
  private final ArmIO io;
  // Generated at build time from ArmIO.ArmIOInputs (see @AutoLog there), so it isn't in src/.
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
  private final Alert motorDisconnectedAlert =
      new Alert("Arm", "MotorDisconnected", "Disconnected arm motor.", Level.HIGH);
  private final Alert encoderDisconnectedAlert =
      new Alert("Arm", "EncoderDisconnected", "Disconnected arm encoder.", Level.HIGH);
  private final MotorFaults.Alerts motorFaultAlerts = new MotorFaults.Alerts("Arm", "Arm");

  // NaN ("not a number") until set while disabled (see periodic), so hold() commands nothing before
  // then.
  private double goalRad = Double.NaN;

  // Drawing for AdvantageScope's Mechanism tab: the measured arm and the goal arm side by side.
  // Origin is the pivot, sized to fit the arm.
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

  // Editable from the dashboard under /Tuning/Arm/ in tuning mode.
  private final TunableNumber kP = new TunableNumber("Arm/kP", ArmConstants.GAINS.kP());
  private final TunableNumber kD = new TunableNumber("Arm/kD", ArmConstants.GAINS.kD());
  private final TunableNumber kG = new TunableNumber("Arm/kG", ArmConstants.GAINS.kG());
  private final TunableNumber cruiseVelocity =
      new TunableNumber("Arm/CruiseVelocity", ArmConstants.GAINS.cruiseVelocity());
  private final TunableNumber acceleration =
      new TunableNumber("Arm/Acceleration", ArmConstants.GAINS.acceleration());
  // What the motor controller currently has, so gains are only re-sent when one changes.
  private ArmIO.Gains appliedGains = ArmConstants.GAINS;

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
    // Never past the soft limits. Clamped here, before the name is built, so the name shows where
    // the arm will really go.
    double clampedRad =
        Math.clamp(angleRad, ArmConstants.SOFT_MIN_ANGLE_RAD, ArmConstants.SOFT_MAX_ANGLE_RAD);
    return run(coroutine -> {
          goalRad = clampedRad;
          while (!isAtGoal()) {
            io.setPosition(goalRad);
            coroutine.yield();
          }
        })
        .named("Arm.GoTo[" + Math.round(Units.radiansToDegrees(clampedRad)) + " deg]");
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
                // A goal outside the soft limits means the arm was resting on a hard stop when the
                // robot enabled (see periodic). Commanding that angle would get it clamped to the
                // soft limit, so the arm would lift a few degrees every time the robot enabled.
                // Instead, leave the motor off and let the arm keep resting on the stop.
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
   * the robot use this to go easier on it (see Controls).
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
