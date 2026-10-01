package first.robot.subsystems.arm;

import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import first.robot.sim.PhoenixSimUtil;
import first.robot.sim.SimulatedMechanism;
import org.wpilib.math.util.Units;
import org.wpilib.simulation.SingleJointedArmSim;

/**
 * Physics for the arm: WPILib's SingleJointedArmSim, a motor and gearbox swinging a rod against
 * gravity between two hard stops. Joins through {@link ArmIOTalonFXSim}.
 */
public class ArmSim implements SimulatedMechanism {
  private final SingleJointedArmSim arm =
      new SingleJointedArmSim(
          ArmConstants.MOTOR.toDCMotor(1),
          ArmConstants.GEAR_RATIO,
          ArmConstants.MOI,
          ArmConstants.LENGTH_METERS,
          ArmConstants.MIN_ANGLE_RAD,
          ArmConstants.MAX_ANGLE_RAD,
          true,
          // Starts resting on the lower hard stop, like a real arm at power-on.
          ArmConstants.MIN_ANGLE_RAD);

  private TalonFXSimState motorSim = null;
  private CANcoderSimState encoderSim = null;

  /** Called by {@link ArmIOTalonFXSim}, before the SimWorld starts. */
  void attach(TalonFXSimState motorSim, CANcoderSimState encoderSim) {
    this.motorSim = motorSim;
    this.encoderSim = encoderSim;
    motorSim.Orientation = PhoenixSimUtil.orientation(ArmConstants.MOTOR_INVERTED);
    motorSim.setMotorType(MotorType.KrakenX60);
    encoderSim.Orientation = PhoenixSimUtil.orientation(ArmConstants.ENCODER_INVERTED);
    encoderSim.SensorOffset = ArmConstants.ENCODER_OFFSET;
    // Seed the sensors so the first reading matches where the model starts.
    writeSensors();
  }

  @Override
  public double update(double dtSeconds, double batteryVolts) {
    motorSim.setSupplyVoltage(batteryVolts);
    encoderSim.setSupplyVoltage(batteryVolts);
    arm.setInputVoltage(motorSim.getMotorVoltage());
    arm.update(dtSeconds);
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
