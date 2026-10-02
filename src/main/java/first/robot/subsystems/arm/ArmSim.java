package first.robot.subsystems.arm;

import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import first.robot.sim.PhoenixSimUtil;
import first.robot.sim.SimulatedMechanism;
import org.wpilib.math.system.Models;
import org.wpilib.math.util.Units;
import org.wpilib.simulation.SingleJointedArmSim;

/**
 * Physics for the arm, built from its characterization constants in ArmConstants, the same ones the
 * gains use. Joins through {@link ArmIOTalonFXSim}.
 *
 * <p>In a characterized model, gravity and friction are both just voltages. Holding the arm
 * horizontal takes kG volts, so gravity acts like kG * cos(angle) volts pushing the other way, and
 * friction swallows the first kS volts. What's left goes to kV and kA, which set how fast the arm
 * speeds up. WPILib's SingleJointedArmSim handles that part and the hard stops.
 */
public class ArmSim implements SimulatedMechanism {
  // FUDGE: real gravity compared with the arm's configured kG. At 1.0 they match. A real arm
  // rarely matches its constants exactly: a game piece on the end, or an extra bracket, makes
  // gravity stronger than kG expects. Try 1.3: at the normal kP the arm holds less than half a
  // degree low, but lower kP to 5 and it's about 4.5 degrees.
  private static final double FUDGE_GRAVITY_SCALE = 1.0;

  private final SingleJointedArmSim arm =
      new SingleJointedArmSim(
          // The plant works in radians, so the per-rotation values are divided by 2 pi.
          Models.singleJointedArmFromSysId(
              ArmConstants.KV / (2 * Math.PI), ArmConstants.KA / (2 * Math.PI)),
          ArmConstants.MOTOR.toDCMotor(1),
          ArmConstants.GEAR_RATIO,
          ArmConstants.LENGTH_METERS,
          ArmConstants.MIN_ANGLE_RAD,
          ArmConstants.MAX_ANGLE_RAD,
          // Gravity comes from kG below instead.
          false,
          // Starts resting on the lower hard stop, like a real arm at power-on.
          ArmConstants.MIN_ANGLE_RAD);

  private TalonFXSimState motorSim = null;
  private CANcoderSimState encoderSim = null;

  /** Called by {@link ArmIOTalonFXSim}, before the SimWorld starts. */
  void attach(TalonFXSimState motorSim, CANcoderSimState encoderSim) {
    this.motorSim = motorSim;
    this.encoderSim = encoderSim;
    motorSim.Orientation = PhoenixSimUtil.orientation(ArmConstants.MOTOR_DIRECTION);
    motorSim.setMotorType(MotorType.KrakenX60);
    encoderSim.Orientation = PhoenixSimUtil.orientation(ArmConstants.ENCODER_DIRECTION);
    encoderSim.SensorOffset = ArmConstants.ENCODER_OFFSET;
    // Seed the sensors so the first reading matches where the model starts.
    writeSensors();
  }

  @Override
  public double update(double dtSeconds, double batteryVolts) {
    motorSim.setSupplyVoltage(batteryVolts);
    encoderSim.setSupplyVoltage(batteryVolts);
    double gravityVolts = FUDGE_GRAVITY_SCALE * ArmConstants.KG * Math.cos(arm.getAngle());
    double netVolts = motorSim.getMotorVoltage() - gravityVolts;
    double before = arm.getVelocity();
    arm.setInputVoltage(PhoenixSimUtil.applyFriction(netVolts, before, ArmConstants.KS));
    arm.update(dtSeconds);
    if (PhoenixSimUtil.stoppedByFriction(before, arm.getVelocity(), netVolts, ArmConstants.KS)) {
      arm.setState(arm.getAngle(), 0.0);
    }
    writeSensors();
    return motorSim.getSupplyCurrent();
  }

  private void writeSensors() {
    double armRotations = Units.radiansToRotations(arm.getAngle());
    double armRotationsPerSec = Units.radiansToRotations(arm.getVelocity());
    motorSim.setRawRotorPosition(armRotations * ArmConstants.GEAR_RATIO);
    motorSim.setRotorVelocity(armRotationsPerSec * ArmConstants.GEAR_RATIO);
    // The CANcoder is on the arm shaft, after the reduction.
    encoderSim.setRawPosition(armRotations);
    encoderSim.setVelocity(armRotationsPerSec);
  }
}
