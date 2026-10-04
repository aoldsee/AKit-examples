package first.robot.subsystems.intake;

import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import first.robot.sim.PhoenixSimUtil;
import first.robot.sim.SimulatedMechanism;
import first.robot.subsystems.intake.IntakeConstants.BeamBreak;
import org.wpilib.math.system.Models;
import org.wpilib.math.util.Units;
import org.wpilib.simulation.DCMotorSim;
import org.wpilib.simulation.DIOSim;

/**
 * Physics for the rollers, plus a very simple game piece.
 *
 * <p>There's no field full of pieces here. Running the rollers inward always finds one, and it's in
 * once the rollers have turned far enough to pull it past the beam break. Running them outward far
 * enough pushes it back out. While a piece is in, it's jammed against a hard stop, so inward
 * voltage stalls the rollers instead of spinning them, the same as on a real robot.
 */
public class IntakeSim implements SimulatedMechanism {
  /** What the simulated robot starts with in its intake. */
  public enum Preload {
    EMPTY,
    ONE_PIECE
  }

  // Roller rotations to pull a piece from first contact to past the sensor, and to push it out.
  private static final double ROTATIONS_TO_LOAD = 3.0;
  private static final double ROTATIONS_TO_EJECT = 3.0;

  // DCMotorSim wants a plant that tracks position as well as speed. Despite the name, the arm
  // model is exactly that (a motor turning an inertia), as long as nothing adds gravity.
  private final DCMotorSim rollers =
      new DCMotorSim(
          Models.singleJointedArmFromPhysicalConstants(
              IntakeConstants.MOTOR.toDCMotor(1),
              IntakeConstants.ROLLER_MOI,
              IntakeConstants.GEAR_RATIO),
          IntakeConstants.MOTOR.toDCMotor(1));

  private TalonFXSimState motorSim = null;
  private DIOSim sensorSim = null;

  // Written by the SimWorld thread, read by tests.
  private volatile boolean hasPiece;
  // How far the piece has traveled into (positive) or out of (negative) the rollers, in roller
  // rotations, since it was last fully in or fully out.
  private double travel = 0.0;

  /**
   * @param preload whether the robot starts holding a piece, as most robots do for auto
   */
  public IntakeSim(Preload preload) {
    hasPiece = preload == Preload.ONE_PIECE;
  }

  /** Called by {@link IntakeIOTalonFXSim}, before the SimWorld starts. */
  void attach(TalonFXSimState motorSim, DIOSim sensorSim) {
    this.motorSim = motorSim;
    this.sensorSim = sensorSim;
    motorSim.Orientation = PhoenixSimUtil.orientation(IntakeConstants.MOTOR_DIRECTION);
    motorSim.setMotorType(MotorType.KrakenX44);
    writeSensor();
  }

  public boolean hasPiece() {
    return hasPiece;
  }

  @Override
  public double update(double dtSeconds, double batteryVolts) {
    motorSim.setSupplyVoltage(batteryVolts);
    double volts = motorSim.getMotorVoltage();
    double positionBefore = rollers.getAngularPosition();
    rollers.setInputVoltage(volts);
    rollers.update(dtSeconds);
    if (hasPiece && volts > 0.0) {
      // Pulling inward against a piece that's already against the stop: nothing turns, so undo
      // this step's movement too. It also pulls a half-ejected piece back in, so an eject that
      // was let go partway starts over next time.
      rollers.setState(positionBefore, 0.0);
      travel = 0.0;
    }

    double rotations = Units.radiansToRotations(rollers.getAngularVelocity()) * dtSeconds;
    if (!hasPiece && rotations > 0.0) {
      travel += rotations;
      if (travel >= ROTATIONS_TO_LOAD) {
        hasPiece = true;
        travel = 0.0;
      }
    } else if (hasPiece && rotations < 0.0) {
      travel += rotations;
      if (travel <= -ROTATIONS_TO_EJECT) {
        hasPiece = false;
        travel = 0.0;
      }
    }

    motorSim.setRawRotorPosition(
        Units.radiansToRotations(rollers.getAngularPosition()) * IntakeConstants.GEAR_RATIO);
    motorSim.setRotorVelocity(
        Units.radiansToRotations(rollers.getAngularVelocity()) * IntakeConstants.GEAR_RATIO);
    writeSensor();
    return motorSim.getSupplyCurrent();
  }

  private void writeSensor() {
    // Set the simulated wire the way the real sensor would read.
    sensorSim.setValue(
        IntakeConstants.BEAM_BREAK == BeamBreak.READS_TRUE_WHEN_BLOCKED ? hasPiece : !hasPiece);
  }
}
