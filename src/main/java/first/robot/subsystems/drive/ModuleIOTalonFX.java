package first.robot.subsystems.drive;

import static first.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import first.robot.generated.TunerConstants;
import first.robot.util.MotorFaults;
import java.util.Queue;
import org.wpilib.math.filter.Debouncer;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.util.Units;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Voltage;

/**
 * Talon FX drive, Talon FX steer, and CANcoder, configured from Phoenix swerve module constants.
 * {@link ModuleIOTalonFXSim} reuses all of this in simulation.
 */
public class ModuleIOTalonFX implements ModuleIO {
  private final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      constants;

  // Protected so the sim subclass can hand these devices to its sim model.
  protected final TalonFX driveTalon;
  protected final TalonFX turnTalon;
  protected final CANcoder cancoder;

  // Two families of requests, picked by DriveMotorClosedLoopOutput and SteerMotorClosedLoopOutput
  // in TunerConstants. Voltage requests set how hard the motor is pushed; the speed it reaches
  // depends on load and battery. TorqueCurrentFOC requests set motor current, which is torque
  // directly, so the response doesn't change as the battery sags. Gains differ between the two
  // (volts per unit vs amps per unit), and so does what an open-loop "output" means: volts in one
  // family, amps in the other.
  // FOC needs Phoenix Pro. Unlicensed Talons report an UnlicensedFeatureInUse fault on these.
  private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(true);
  private final PositionVoltage positionVoltageRequest =
      new PositionVoltage(0.0).withEnableFOC(true);
  private final VelocityVoltage velocityVoltageRequest =
      new VelocityVoltage(0.0).withEnableFOC(true);
  private final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0);
  private final PositionTorqueCurrentFOC positionTorqueCurrentRequest =
      new PositionTorqueCurrentFOC(0.0);
  private final VelocityTorqueCurrentFOC velocityTorqueCurrentRequest =
      new VelocityTorqueCurrentFOC(0.0);

  // Wheel rotations per module rotation. The MK5n bevel drives the wheel when the module steers,
  // so a pure steer reads as drive travel unless this is backed out.
  private final double couplingWheelRotPerTurnRot;

  private final Queue<Double> timestampQueue;

  private final StatusSignal<Angle> drivePosition;
  private final Queue<Double> drivePositionQueue;
  private final StatusSignal<AngularVelocity> driveVelocity;
  private final StatusSignal<Voltage> driveAppliedVolts;
  private final StatusSignal<Current> driveCurrent;

  private final StatusSignal<Angle> turnAbsolutePosition;
  private final StatusSignal<Angle> turnPosition;
  private final Queue<Double> turnPositionQueue;
  private final StatusSignal<AngularVelocity> turnVelocity;
  private final StatusSignal<Voltage> turnAppliedVolts;
  private final StatusSignal<Current> turnCurrent;

  private final MotorFaults.Signals driveFaults;
  private final MotorFaults.Signals turnFaults;

  // A single dropped frame shouldn't raise a disconnect alert.
  private final Debouncer driveConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.FALLING);
  private final Debouncer turnConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.FALLING);
  private final Debouncer turnEncoderConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.FALLING);

  public ModuleIOTalonFX(
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
          constants) {
    this.constants = constants;
    couplingWheelRotPerTurnRot = constants.CouplingGearRatio / constants.DriveMotorGearRatio;
    driveTalon = new TalonFX(constants.DriveMotorId, TunerConstants.kCANBus);
    turnTalon = new TalonFX(constants.SteerMotorId, TunerConstants.kCANBus);
    cancoder = new CANcoder(constants.EncoderId, TunerConstants.kCANBus);

    // Drive positions and velocities come out in wheel rotations because of SensorToMechanismRatio.
    var driveConfig = constants.DriveMotorInitialConfigs;
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    driveConfig.Slot0 = constants.DriveMotorGains;
    driveConfig.Feedback.SensorToMechanismRatio = constants.DriveMotorGearRatio;
    driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = constants.SlipCurrent;
    driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = -constants.SlipCurrent;
    driveConfig.CurrentLimits.StatorCurrentLimit = constants.SlipCurrent;
    driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    driveConfig.MotorOutput.Inverted =
        constants.DriveMotorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    tryUntilOk(5, () -> driveTalon.getConfigurator().apply(driveConfig, 0.25));
    // Wheel distance only matters as change from one sample to the next, so any starting value
    // works. Zero keeps the logged positions easy to read.
    tryUntilOk(5, () -> driveTalon.setPosition(0.0, 0.25));

    // Fused/Sync CANcoder close the loop on the absolute encoder, so turn positions are module
    // rotations and RotorToSensorRatio carries the steer reduction.
    var turnConfig = new TalonFXConfiguration();
    turnConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    turnConfig.Slot0 = constants.SteerMotorGains;
    turnConfig.Feedback.FeedbackRemoteSensorID = constants.EncoderId;
    turnConfig.Feedback.FeedbackSensorSource =
        switch (constants.FeedbackSource) {
          case RemoteCANcoder -> FeedbackSensorSourceValue.RemoteCANcoder;
          case FusedCANcoder -> FeedbackSensorSourceValue.FusedCANcoder;
          case SyncCANcoder -> FeedbackSensorSourceValue.SyncCANcoder;
          default ->
              throw new IllegalStateException(
                  "ModuleIOTalonFX supports only CANcoder steer feedback, got "
                      + constants.FeedbackSource);
        };
    turnConfig.Feedback.RotorToSensorRatio = constants.SteerMotorGearRatio;
    turnConfig.ClosedLoopGeneral.ContinuousWrap = true;
    turnConfig.MotorOutput.Inverted =
        constants.SteerMotorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    tryUntilOk(5, () -> turnTalon.getConfigurator().apply(turnConfig, 0.25));

    CANcoderConfiguration cancoderConfig = constants.EncoderInitialConfigs;
    cancoderConfig.MagnetSensor.MagnetOffset = constants.EncoderOffset;
    cancoderConfig.MagnetSensor.SensorDirection =
        constants.EncoderInverted
            ? SensorDirectionValue.Clockwise_Positive
            : SensorDirectionValue.CounterClockwise_Positive;
    tryUntilOk(5, () -> cancoder.getConfigurator().apply(cancoderConfig, 0.25));

    timestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();

    // false skips the refresh Phoenix normally does on lookup. Right after startup the first status
    // frame may not have arrived, and that refresh would log a CAN error. updateInputs refreshes
    // every loop anyway.
    drivePosition = driveTalon.getPosition(false);
    drivePositionQueue = PhoenixOdometryThread.getInstance().registerSignal(drivePosition.clone());
    driveVelocity = driveTalon.getVelocity(false);
    driveAppliedVolts = driveTalon.getMotorVoltage(false);
    driveCurrent = driveTalon.getStatorCurrent(false);

    turnAbsolutePosition = cancoder.getAbsolutePosition(false);
    turnPosition = turnTalon.getPosition(false);
    turnPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(turnPosition.clone());
    turnVelocity = turnTalon.getVelocity(false);
    turnAppliedVolts = turnTalon.getMotorVoltage(false);
    turnCurrent = turnTalon.getStatorCurrent(false);

    // Only the two positions odometry uses need the high rate. Everything else is read once per
    // 20 ms loop, so faster frames would only crowd the CAN bus.
    BaseStatusSignal.setUpdateFrequencyForAll(
        DriveConstants.ODOMETRY_FREQUENCY, drivePosition, turnPosition);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        driveVelocity,
        driveAppliedVolts,
        driveCurrent,
        turnAbsolutePosition,
        turnVelocity,
        turnAppliedVolts,
        turnCurrent);
    driveFaults = new MotorFaults.Signals(driveTalon);
    turnFaults = new MotorFaults.Signals(turnTalon);
    // Turns off every status frame not given an explicit rate above.
    ParentDevice.optimizeBusUtilizationForAll(driveTalon, turnTalon);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    var driveStatus =
        BaseStatusSignal.refreshAll(drivePosition, driveVelocity, driveAppliedVolts, driveCurrent);
    var turnStatus =
        BaseStatusSignal.refreshAll(turnPosition, turnVelocity, turnAppliedVolts, turnCurrent);
    var turnEncoderStatus = BaseStatusSignal.refreshAll(turnAbsolutePosition);

    inputs.driveConnected = driveConnectedDebounce.calculate(driveStatus.isOK());
    // turnPosition is continuous across turns, so this stays correct after many rotations.
    inputs.drivePositionRad =
        Units.rotationsToRadians(
            drivePosition.getValueAsDouble()
                - turnPosition.getValueAsDouble() * couplingWheelRotPerTurnRot);
    inputs.driveVelocityRadPerSec =
        Units.rotationsToRadians(
            driveVelocity.getValueAsDouble()
                - turnVelocity.getValueAsDouble() * couplingWheelRotPerTurnRot);
    inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
    inputs.driveCurrentAmps = driveCurrent.getValueAsDouble();
    inputs.driveFaults = driveFaults.read();

    inputs.turnConnected = turnConnectedDebounce.calculate(turnStatus.isOK());
    inputs.turnEncoderConnected = turnEncoderConnectedDebounce.calculate(turnEncoderStatus.isOK());
    inputs.turnAbsolutePosition = Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble());
    inputs.turnPosition = Rotation2d.fromRotations(turnPosition.getValueAsDouble());
    inputs.turnVelocityRadPerSec = Units.rotationsToRadians(turnVelocity.getValueAsDouble());
    inputs.turnAppliedVolts = turnAppliedVolts.getValueAsDouble();
    inputs.turnCurrentAmps = turnCurrent.getValueAsDouble();
    inputs.turnFaults = turnFaults.read();

    inputs.odometryTimestamps = timestampQueue.stream().mapToDouble(v -> v).toArray();
    // The thread samples drive and turn together, so the queues share indices.
    double[] odometryTurnRotations = turnPositionQueue.stream().mapToDouble(v -> v).toArray();
    double[] odometryDriveRotations = drivePositionQueue.stream().mapToDouble(v -> v).toArray();
    int sampleCount = Math.min(odometryTurnRotations.length, odometryDriveRotations.length);
    inputs.odometryDrivePositionsRad = new double[sampleCount];
    inputs.odometryTurnPositions = new Rotation2d[sampleCount];
    for (int i = 0; i < sampleCount; i++) {
      inputs.odometryDrivePositionsRad[i] =
          Units.rotationsToRadians(
              odometryDriveRotations[i] - odometryTurnRotations[i] * couplingWheelRotPerTurnRot);
      inputs.odometryTurnPositions[i] = Rotation2d.fromRotations(odometryTurnRotations[i]);
    }
    timestampQueue.clear();
    drivePositionQueue.clear();
    turnPositionQueue.clear();
  }

  @Override
  public void setDriveOpenLoop(double output) {
    driveTalon.setControl(
        switch (constants.DriveMotorClosedLoopOutput) {
          case Voltage -> voltageRequest.withOutput(output);
          case TorqueCurrentFOC -> torqueCurrentRequest.withOutput(output);
        });
  }

  @Override
  public void setTurnOpenLoop(double output) {
    turnTalon.setControl(
        switch (constants.SteerMotorClosedLoopOutput) {
          case Voltage -> voltageRequest.withOutput(output);
          case TorqueCurrentFOC -> torqueCurrentRequest.withOutput(output);
        });
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    // Feed forward the coupled motion so the wheel, not the motor, holds the requested speed.
    double velocityRotPerSec =
        Units.radiansToRotations(velocityRadPerSec)
            + turnVelocity.getValueAsDouble() * couplingWheelRotPerTurnRot;
    driveTalon.setControl(
        switch (constants.DriveMotorClosedLoopOutput) {
          case Voltage -> velocityVoltageRequest.withVelocity(velocityRotPerSec);
          case TorqueCurrentFOC -> velocityTorqueCurrentRequest.withVelocity(velocityRotPerSec);
        });
  }

  @Override
  public void setTurnPosition(Rotation2d rotation) {
    turnTalon.setControl(
        switch (constants.SteerMotorClosedLoopOutput) {
          case Voltage -> positionVoltageRequest.withPosition(rotation.getRotations());
          case TorqueCurrentFOC ->
              positionTorqueCurrentRequest.withPosition(rotation.getRotations());
        });
  }
}
