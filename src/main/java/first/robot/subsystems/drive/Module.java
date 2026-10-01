package first.robot.subsystems.drive;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import first.robot.util.MotorFaults;
import org.littletonrobotics.junction.Logger;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.math.util.Units;
import org.wpilib.util.Alert;
import org.wpilib.util.Alert.Level;

/** One swerve module. Converts between wheel radians from the IO layer and meters. */
public class Module {
  private final ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  private final int index;
  private final double wheelRadiusMeters;

  private final Alert driveDisconnectedAlert;
  private final Alert turnDisconnectedAlert;
  private final Alert turnEncoderDisconnectedAlert;
  private final MotorFaults.Alerts driveFaultAlerts;
  private final MotorFaults.Alerts turnFaultAlerts;
  private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};

  public Module(
      ModuleIO io,
      int index,
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
          constants) {
    this.io = io;
    this.index = index;
    this.wheelRadiusMeters = constants.WheelRadius;
    driveDisconnectedAlert =
        new Alert(
            "Drive",
            "Module" + index + "/DriveDisconnected",
            "Disconnected drive motor on module " + index + ".",
            Level.HIGH);
    turnDisconnectedAlert =
        new Alert(
            "Drive",
            "Module" + index + "/TurnDisconnected",
            "Disconnected turn motor on module " + index + ".",
            Level.HIGH);
    turnEncoderDisconnectedAlert =
        new Alert(
            "Drive",
            "Module" + index + "/TurnEncoderDisconnected",
            "Disconnected turn encoder on module " + index + ".",
            Level.HIGH);
    driveFaultAlerts = new MotorFaults.Alerts("Drive", "Module" + index + "/Drive");
    turnFaultAlerts = new MotorFaults.Alerts("Drive", "Module" + index + "/Turn");
  }

  /** Call with {@link Drive#odometryLock} held so the odometry queues drain consistently. */
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Drive/Module" + index, inputs);

    int sampleCount = inputs.odometryTimestamps.length;
    odometryPositions = new SwerveModulePosition[sampleCount];
    for (int i = 0; i < sampleCount; i++) {
      odometryPositions[i] =
          new SwerveModulePosition(
              inputs.odometryDrivePositionsRad[i] * wheelRadiusMeters,
              inputs.odometryTurnPositions[i]);
    }

    driveDisconnectedAlert.set(!inputs.driveConnected);
    turnDisconnectedAlert.set(!inputs.turnConnected);
    turnEncoderDisconnectedAlert.set(!inputs.turnEncoderConnected);
    driveFaultAlerts.update(inputs.driveFaults);
    turnFaultAlerts.update(inputs.turnFaults);
  }

  /**
   * Flips the setpoint if that's a shorter turn, then scales speed by the cosine of the remaining
   * angle error so the wheel doesn't push sideways while it's still turning.
   *
   * @return the setpoint actually sent, for logging
   */
  public SwerveModuleVelocity runSetpoint(SwerveModuleVelocity setpoint) {
    var optimized = setpoint.optimize(getAngle()).cosineScale(getAngle());
    io.setDriveVelocity(optimized.velocity / wheelRadiusMeters);
    io.setTurnPosition(optimized.angle);
    return optimized;
  }

  /** Open-loop drive output with the module held straight ahead, for feedforward tests. */
  public void runCharacterization(double output) {
    runDirect(Rotation2d.ZERO, output);
  }

  /**
   * Points the module at {@code angle} and runs the drive open loop, with none of runSetpoint's
   * flipping or slowing. For characterization and the systems check, which need to know exactly
   * what each motor was told.
   */
  public void runDirect(Rotation2d angle, double driveOutput) {
    io.setDriveOpenLoop(driveOutput);
    io.setTurnPosition(angle);
  }

  public void stop() {
    io.setDriveOpenLoop(0.0);
    io.setTurnOpenLoop(0.0);
  }

  public Rotation2d getAngle() {
    return inputs.turnPosition;
  }

  public double getPositionMeters() {
    return inputs.drivePositionRad * wheelRadiusMeters;
  }

  public double getVelocityMetersPerSec() {
    return inputs.driveVelocityRadPerSec * wheelRadiusMeters;
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), getAngle());
  }

  public SwerveModuleVelocity getVelocity() {
    return new SwerveModuleVelocity(getVelocityMetersPerSec(), getAngle());
  }

  /** Positions sampled by the odometry thread since the last cycle. */
  public SwerveModulePosition[] getOdometryPositions() {
    return odometryPositions;
  }

  public double[] getOdometryTimestamps() {
    return inputs.odometryTimestamps;
  }

  /** Wheel radians. Wheel radius characterization needs angle, not distance. */
  public double getWheelRadiusCharacterizationPosition() {
    return inputs.drivePositionRad;
  }

  /** Wheel rotations/s, matching the units of Phoenix drive kV. */
  public double getFFCharacterizationVelocity() {
    return Units.radiansToRotations(inputs.driveVelocityRadPerSec);
  }
}
