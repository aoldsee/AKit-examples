package first.robot.subsystems.drive;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import first.robot.sim.PhoenixSimUtil;
import first.robot.util.Motors;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.Models;
import org.wpilib.math.util.Units;
import org.wpilib.simulation.DCMotorSim;

/**
 * Physics for one swerve module: a drive motor turning the wheel and a steer motor turning the
 * module, each modeled as a motor plus inertia behind its gear reduction.
 *
 * <p>Models the MK5n bevel coupling (steering also turns the drive motor) so ModuleIOTalonFX's
 * compensation is exercised. Coupling is kinematic only; the steer motor doesn't feel the drive
 * load through the bevel.
 */
class SwerveModuleSim {
  private final TalonFXSimState driveSim;
  private final TalonFXSimState steerSim;
  private final CANcoderSimState encoderSim;
  private final DCMotorSim driveMotor;
  private final DCMotorSim steerMotor;
  private final double driveGearing;
  private final double steerGearing;
  private final double couplingRatio;
  private final double driveFrictionVolts;
  private final double steerFrictionVolts;
  private final double wheelRadiusMeters;
  private final Translation2d location;
  private double supplyCurrentAmps = 0.0;

  SwerveModuleSim(
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
          constants,
      TalonFXSimState driveSim,
      TalonFXSimState steerSim,
      CANcoderSimState encoderSim) {
    this.driveSim = driveSim;
    this.steerSim = steerSim;
    this.encoderSim = encoderSim;

    // Same constants ModuleIOTalonFX configured the real devices from, so inverts always agree.
    driveSim.Orientation = PhoenixSimUtil.orientation(constants.DriveMotorInverted);
    steerSim.Orientation = PhoenixSimUtil.orientation(constants.SteerMotorInverted);
    encoderSim.Orientation = PhoenixSimUtil.orientation(constants.EncoderInverted);
    encoderSim.SensorOffset = constants.EncoderOffset;
    // TunerConstants can't tell an X60 from an X44 (both are TalonFX_Integrated), so the MK5n's
    // motors are stated here.
    driveSim.setMotorType(MotorType.KrakenX60);
    steerSim.setMotorType(MotorType.KrakenX44);

    // Same specs TunerConstants derives its gains from, so the sim and the gains agree.
    DCMotor driveGearbox = Motors.KRAKEN_X60_FOC.toDCMotor(1);
    DCMotor steerGearbox = Motors.KRAKEN_X44_FOC.toDCMotor(1);
    // The models are in mechanism units (wheel and module angle), so the gearing goes into the
    // plant and the rotor values are multiplied back out in update().
    driveMotor =
        new DCMotorSim(
            Models.singleJointedArmFromPhysicalConstants(
                driveGearbox, constants.DriveInertia, constants.DriveMotorGearRatio),
            driveGearbox);
    steerMotor =
        new DCMotorSim(
            Models.singleJointedArmFromPhysicalConstants(
                steerGearbox, constants.SteerInertia, constants.SteerMotorGearRatio),
            steerGearbox);

    driveGearing = constants.DriveMotorGearRatio;
    steerGearing = constants.SteerMotorGearRatio;
    couplingRatio = constants.CouplingGearRatio;
    driveFrictionVolts = constants.DriveFrictionVoltage;
    steerFrictionVolts = constants.SteerFrictionVoltage;
    wheelRadiusMeters = constants.WheelRadius;
    location = new Translation2d(constants.LocationX, constants.LocationY);
  }

  /** Battery current both motors drew during the last update. */
  double getSupplyCurrentAmps() {
    return supplyCurrentAmps;
  }

  Translation2d getLocation() {
    return location;
  }

  /** Advances the module and returns its true velocity. */
  SwerveModuleVelocity update(double dtSeconds, double batteryVolts) {
    driveSim.setSupplyVoltage(batteryVolts);
    steerSim.setSupplyVoltage(batteryVolts);
    encoderSim.setSupplyVoltage(batteryVolts);

    driveMotor.setInputVoltage(
        PhoenixSimUtil.applyFriction(driveSim.getMotorVoltage(), driveFrictionVolts));
    steerMotor.setInputVoltage(
        PhoenixSimUtil.applyFriction(steerSim.getMotorVoltage(), steerFrictionVolts));
    driveMotor.update(dtSeconds);
    steerMotor.update(dtSeconds);

    // Sim state setters take rotations, the DCMotorSim getters give radians.
    // The drive rotor turns with the wheel and also with the module through the bevel.
    driveSim.setRawRotorPosition(
        Units.radiansToRotations(
            driveMotor.getAngularPosition() * driveGearing
                + steerMotor.getAngularPosition() * couplingRatio));
    driveSim.setRotorVelocity(
        Units.radiansToRotations(
            driveMotor.getAngularVelocity() * driveGearing
                + steerMotor.getAngularVelocity() * couplingRatio));
    driveSim.setRotorAcceleration(
        Units.radiansToRotations(driveMotor.getAngularAcceleration() * driveGearing));

    steerSim.setRawRotorPosition(
        Units.radiansToRotations(steerMotor.getAngularPosition() * steerGearing));
    steerSim.setRotorVelocity(
        Units.radiansToRotations(steerMotor.getAngularVelocity() * steerGearing));
    steerSim.setRotorAcceleration(
        Units.radiansToRotations(steerMotor.getAngularAcceleration() * steerGearing));

    // The CANcoder sits on the module, after the steer reduction.
    encoderSim.setRawPosition(Units.radiansToRotations(steerMotor.getAngularPosition()));
    encoderSim.setVelocity(Units.radiansToRotations(steerMotor.getAngularVelocity()));

    // Phoenix's sim works out each Talon's battery current from its output and motor speed.
    supplyCurrentAmps = driveSim.getSupplyCurrent() + steerSim.getSupplyCurrent();

    return new SwerveModuleVelocity(
        driveMotor.getAngularVelocity() * wheelRadiusMeters,
        Rotation2d.fromRadians(steerMotor.getAngularPosition()));
  }
}
