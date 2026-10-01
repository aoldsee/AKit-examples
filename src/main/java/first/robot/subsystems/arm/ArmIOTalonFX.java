package first.robot.subsystems.arm;

import static first.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import first.robot.util.MotorFaults;
import org.wpilib.math.filter.Debouncer;
import org.wpilib.math.util.Units;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Voltage;

/**
 * Talon FX with a CANcoder on the arm shaft. The Talon fuses the two (Phoenix Pro), so it reports
 * arm rotations directly and runs Motion Magic and gravity feedforward on board. {@link
 * ArmIOTalonFXSim} reuses all of this in simulation.
 */
public class ArmIOTalonFX implements ArmIO {
  // Protected so the sim subclass can hand these devices to its sim model.
  protected final TalonFX motor = new TalonFX(ArmConstants.MOTOR_ID, ArmConstants.CAN_BUS);
  protected final CANcoder encoder = new CANcoder(ArmConstants.ENCODER_ID, ArmConstants.CAN_BUS);

  // Motion Magic is a motion profile that runs on the Talon itself. Given a target, the Talon
  // plans a smooth speed-up, cruise, and slow-down within the cruise velocity and acceleration
  // limits, then follows that plan instead of jumping straight at the target.
  private final MotionMagicVoltage positionRequest =
      new MotionMagicVoltage(0.0).withEnableFOC(true);
  private final VoltageOut voltageRequest = new VoltageOut(0.0).withEnableFOC(true);

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> current;
  private final StatusSignal<Angle> absolutePosition;
  private final MotorFaults.Signals motorFaults;

  // A single dropped frame shouldn't raise a disconnect alert.
  private final Debouncer motorConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.FALLING);
  private final Debouncer encoderConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.FALLING);

  public ArmIOTalonFX() {
    var encoderConfig = new CANcoderConfiguration();
    encoderConfig.MagnetSensor.MagnetOffset = ArmConstants.ENCODER_OFFSET;
    encoderConfig.MagnetSensor.SensorDirection =
        ArmConstants.ENCODER_INVERTED
            ? SensorDirectionValue.Clockwise_Positive
            : SensorDirectionValue.CounterClockwise_Positive;
    tryUntilOk(5, () -> encoder.getConfigurator().apply(encoderConfig, 0.25));

    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted =
        ArmConstants.MOTOR_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    config.CurrentLimits.StatorCurrentLimit = ArmConstants.STATOR_CURRENT_LIMIT_AMPS;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    // The CANcoder is on the arm, so it already reads arm rotations. RotorToSensorRatio tells the
    // Talon how far the motor turns per CANcoder turn, which it needs to fuse the two.
    config.Feedback.FeedbackRemoteSensorID = ArmConstants.ENCODER_ID;
    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    config.Feedback.RotorToSensorRatio = ArmConstants.GEAR_RATIO;
    config.Feedback.SensorToMechanismRatio = 1.0;

    config.Slot0 = slot0For(ArmConstants.GAINS);
    config.MotionMagic = motionMagicFor(ArmConstants.GAINS);

    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        Units.radiansToRotations(ArmConstants.SOFT_MAX_ANGLE_RAD);
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        Units.radiansToRotations(ArmConstants.SOFT_MIN_ANGLE_RAD);
    tryUntilOk(5, () -> motor.getConfigurator().apply(config, 0.25));

    // false skips the refresh Phoenix normally does on lookup. Right after startup the first status
    // frame may not have arrived, and that refresh would log a CAN error. updateInputs refreshes
    // every loop anyway.
    position = motor.getPosition(false);
    velocity = motor.getVelocity(false);
    appliedVolts = motor.getMotorVoltage(false);
    current = motor.getStatorCurrent(false);
    absolutePosition = encoder.getAbsolutePosition(false);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, position, velocity, appliedVolts, current, absolutePosition);
    motorFaults = new MotorFaults.Signals(motor);
    // Turns off every status frame not given an explicit rate above.
    ParentDevice.optimizeBusUtilizationForAll(motor, encoder);
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    var motorStatus = BaseStatusSignal.refreshAll(position, velocity, appliedVolts, current);
    var encoderStatus = BaseStatusSignal.refreshAll(absolutePosition);

    inputs.motorConnected = motorConnectedDebounce.calculate(motorStatus.isOK());
    inputs.encoderConnected = encoderConnectedDebounce.calculate(encoderStatus.isOK());
    inputs.positionRad = Units.rotationsToRadians(position.getValueAsDouble());
    inputs.absolutePositionRad = Units.rotationsToRadians(absolutePosition.getValueAsDouble());
    inputs.velocityRadPerSec = Units.rotationsToRadians(velocity.getValueAsDouble());
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.currentAmps = current.getValueAsDouble();
    inputs.motorFaults = motorFaults.read();
  }

  @Override
  public void setGains(Gains gains) {
    // Applying only these two groups leaves the rest of the config untouched. Each apply waits
    // for the Talon to confirm, which is fine for a rare tuning change but not for every loop.
    motor.getConfigurator().apply(slot0For(gains));
    motor.getConfigurator().apply(motionMagicFor(gains));
  }

  // Shared by the constructor and setGains, so tuned and startup gains are built the same way.
  // kS, kV, and kA come from motor physics rather than tuning, so they stay constant.
  private static Slot0Configs slot0For(Gains gains) {
    return new Slot0Configs()
        // Arm_Cosine scales kG by cos(position), which is why 0 must be horizontal.
        .withGravityType(GravityTypeValue.Arm_Cosine)
        .withKG(gains.kG())
        .withKS(ArmConstants.KS)
        .withKV(ArmConstants.KV)
        .withKA(ArmConstants.KA)
        .withKP(gains.kP())
        .withKD(gains.kD());
  }

  private static MotionMagicConfigs motionMagicFor(Gains gains) {
    return new MotionMagicConfigs()
        .withMotionMagicCruiseVelocity(gains.cruiseVelocity())
        .withMotionMagicAcceleration(gains.acceleration());
  }

  @Override
  public void setPosition(double angleRad) {
    motor.setControl(positionRequest.withPosition(Units.radiansToRotations(angleRad)));
  }

  @Override
  public void setVoltage(double volts) {
    motor.setControl(voltageRequest.withOutput(volts));
  }
}
