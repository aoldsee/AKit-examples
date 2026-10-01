package first.robot.subsystems.intake;

import static first.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import first.robot.util.MotorFaults;
import org.wpilib.hardware.discrete.DigitalInput;
import org.wpilib.math.filter.Debouncer;
import org.wpilib.math.util.Units;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Voltage;

/**
 * Talon FX driving the rollers, and a beam break on a SystemCore digital input. {@link
 * IntakeIOTalonFXSim} reuses all of this in simulation.
 */
public class IntakeIOTalonFX implements IntakeIO {
  // Protected so the sim subclass can hand these to its sim model.
  protected final TalonFX motor = new TalonFX(IntakeConstants.MOTOR_ID, IntakeConstants.CAN_BUS);
  protected final DigitalInput sensor = new DigitalInput(IntakeConstants.SENSOR_CHANNEL);

  private final VoltageOut voltageRequest = new VoltageOut(0.0).withEnableFOC(true);

  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> current;
  private final MotorFaults.Signals motorFaults;

  // A single dropped frame shouldn't raise a disconnect alert.
  private final Debouncer motorConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.FALLING);

  public IntakeIOTalonFX() {
    var config = new TalonFXConfiguration();
    // Coast lets a jammed piece be pulled out by hand while disabled.
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted =
        IntakeConstants.MOTOR_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    // Rollers spend a lot of time stalled against a piece. The limit keeps that from cooking the
    // motor or browning out the robot.
    config.CurrentLimits.StatorCurrentLimit = IntakeConstants.STATOR_CURRENT_LIMIT_AMPS;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.Feedback.SensorToMechanismRatio = IntakeConstants.GEAR_RATIO;
    tryUntilOk(5, () -> motor.getConfigurator().apply(config, 0.25));

    // false skips the refresh Phoenix normally does on lookup. Right after startup the first status
    // frame may not have arrived, and that refresh would log a CAN error. updateInputs refreshes
    // every loop anyway.
    velocity = motor.getVelocity(false);
    appliedVolts = motor.getMotorVoltage(false);
    current = motor.getStatorCurrent(false);
    BaseStatusSignal.setUpdateFrequencyForAll(50.0, velocity, appliedVolts, current);
    motorFaults = new MotorFaults.Signals(motor);
    // Turns off every status frame not given an explicit rate above.
    ParentDevice.optimizeBusUtilizationForAll(motor);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    var status = BaseStatusSignal.refreshAll(velocity, appliedVolts, current);
    inputs.motorConnected = motorConnectedDebounce.calculate(status.isOK());
    // XOR: flips the reading when the sensor is inverted, passes it through when not.
    inputs.sensorBlocked = sensor.get() ^ IntakeConstants.SENSOR_INVERTED;
    inputs.velocityRadPerSec = Units.rotationsToRadians(velocity.getValueAsDouble());
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.currentAmps = current.getValueAsDouble();
    inputs.motorFaults = motorFaults.read();
  }

  @Override
  public void setVoltage(double volts) {
    motor.setControl(voltageRequest.withOutput(volts));
  }
}
