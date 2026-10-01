package first.robot.generated;

import static org.wpilib.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;
import first.robot.Constants;
import first.robot.util.MK5n;
import first.robot.util.Motors;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.LinearVelocity;
import org.wpilib.units.measure.MomentOfInertia;
import org.wpilib.units.measure.Voltage;

/**
 * Swerve constants laid out the way the Tuner X swerve generator writes them, so a generated file
 * can replace this one with minimal edits. Tuner X could not generate for 2027 alpha-7 when this
 * was written, so this describes a placeholder robot: SDS MK5n R2 modules with Kraken X60 drive and
 * Kraken X44 steer motors, CANcoders, and a Pigeon 2.
 *
 * <p>Unlike generator output, the gear ratios, wheel size, and feedforward gains come from the
 * hardware specs in {@link MK5n} and {@link Motors} rather than being typed in, so changing a
 * module or motor there updates everything here.
 *
 * <p>Differences from 2026 generator output: {@link CANBus} is built from a SystemCore {@link
 * CANPort} and takes no hoot log path, and the drivetrain uses {@code withNetwork} instead of
 * {@code withCANBusName}.
 */
public final class TunerConstants {
  private TunerConstants() {}

  // Steer gains are in volts per module rotation because FusedCANcoder makes the CANcoder the
  // feedback sensor. kV is the volts to spin the module at 1 rotation/s: the motor turns
  // rpsPerVolt per volt, and the module turns 1 / STEER_RATIO as fast as the motor.
  private static final Slot0Configs steerGains =
      new Slot0Configs()
          .withKP(100)
          .withKI(0)
          .withKD(0.5)
          .withKS(0.1)
          .withKV(MK5n.STEER_RATIO / Motors.KRAKEN_X44_FOC.rpsPerVolt())
          .withKA(0)
          .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

  // Drive gains are in volts per wheel rotation, because ModuleIOTalonFX sets
  // SensorToMechanismRatio to the drive reduction. kV is worked out the same way as steer. kS is
  // what feedforward characterization measures in sim (the simulated friction). Without it, the
  // tiny speeds asked for in the last centimeter of a DriveToPose don't overcome friction, and the
  // robot stalls just short of the target.
  //
  // kP starts from the generator's default of 0.1 V per rotation/s of error, which is per *motor*
  // rotation. One wheel rotation is DRIVE_RATIO motor rotations, so the same error measured in
  // wheel rotations is a number 6 times smaller. Multiplying by the ratio keeps the motor pushing
  // just as hard for the same speed error. Any gain converted between motor and mechanism units
  // needs this
  private static final Slot0Configs driveGains =
      new Slot0Configs()
          .withKP(0.1 * MK5n.DRIVE_RATIO_R2)
          .withKI(0)
          .withKD(0)
          .withKS(0.2)
          .withKV(MK5n.DRIVE_RATIO_R2 / Motors.KRAKEN_X60_FOC.rpsPerVolt());

  private static final ClosedLoopOutputType kSteerClosedLoopOutput = ClosedLoopOutputType.Voltage;
  private static final ClosedLoopOutputType kDriveClosedLoopOutput = ClosedLoopOutputType.Voltage;

  private static final DriveMotorArrangement kDriveMotorType =
      DriveMotorArrangement.TalonFX_Integrated;
  private static final SteerMotorArrangement kSteerMotorType =
      SteerMotorArrangement.TalonFX_Integrated;

  // Fused and Sync CANcoder need Phoenix Pro. Without a license they silently fall back to
  // RemoteCANcoder.
  private static final SteerFeedbackType kSteerFeedbackType = SteerFeedbackType.FusedCANcoder;

  // Phoenix applies this as the drive stator current limit. Spike wheels (MK5n.WHEEL_COF) would
  // not slip until roughly 178 A at 74 kg, so 120 A is a deliberate traction and brownout cap.
  private static final Current kSlipCurrent = Amps.of(120.0);

  // Phoenix overwrites some fields of these (inverts, feedback source, gains). See the
  // with*InitialConfigs() docs before relying on a value set here.
  // Supply current is what the battery sees, and it's what causes brownouts. A full-throttle
  // launch in sim bottoms out at:
  //   70 A (Phoenix's default): 7.7 V on a fresh battery, 4.6 V on a worn one (brownout)
  //   40 A: 9.6 V fresh, 8.1 V worn, 7.1 V worn and half charged
  // The cost of a lower limit is a slower launch. See DriveSimTest.hardAccelerationDoesNotBrownOut,
  // and try Battery.RESISTANCE_OHMS = 0.030 (worn) or a half-charged new Battery(18.0, 0.5).
  private static final Current kDriveSupplyCurrentLimit = Amps.of(40);

  private static final TalonFXConfiguration driveInitialConfigs =
      new TalonFXConfiguration()
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withSupplyCurrentLimit(kDriveSupplyCurrentLimit)
                  // Phoenix normally drops to a lower limit after a second; keep it simple.
                  .withSupplyCurrentLowerLimit(kDriveSupplyCurrentLimit)
                  .withSupplyCurrentLimitEnable(true));
  private static final TalonFXConfiguration steerInitialConfigs =
      new TalonFXConfiguration()
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  // Azimuth needs little torque, and a low limit cuts brownout risk.
                  .withStatorCurrentLimit(Amps.of(60))
                  .withStatorCurrentLimitEnable(true));
  private static final CANcoderConfiguration encoderInitialConfigs = new CANcoderConfiguration();
  // Null skips applying Pigeon 2 configs.
  private static final Pigeon2Configuration pigeonConfigs = null;

  // All swerve devices must share one bus. CAN_S0 is a SystemCore built-in CAN FD port, which
  // gives 250 Hz odometry.
  public static final CANBus kCANBus = new CANBus(CANPort.CAN_S0);

  // Theoretical top speed: motor free speed, through the reduction, times wheel circumference.
  public static final LinearVelocity kSpeedAt12Volts =
      MetersPerSecond.of(
          Motors.KRAKEN_X60_FOC.freeSpeedRps()
              / MK5n.DRIVE_RATIO_R2
              * 2
              * Math.PI
              * MK5n.WHEEL_RADIUS_METERS);

  private static final double kCoupleRatio = MK5n.COUPLING_RATIO;
  private static final double kDriveGearRatio = MK5n.DRIVE_RATIO_R2;
  private static final double kSteerGearRatio = MK5n.STEER_RATIO;
  private static final Distance kWheelRadius = Meters.of(MK5n.WHEEL_RADIUS_METERS);

  private static final boolean kInvertLeftSide = false;
  private static final boolean kInvertRightSide = true;

  private static final int kPigeonId = 13;

  // Used only by simulation.
  private static final MomentOfInertia kSteerInertia = KilogramSquareMeters.of(0.004);
  // What each drive motor really has to accelerate is a quarter of the robot. A mass m moving with
  // the edge of a wheel of radius r resists like an inertia of m * r^2. Exact for driving straight;
  // close enough for turning.
  private static final MomentOfInertia kDriveInertia =
      KilogramSquareMeters.of(
          Constants.robotMassKg / 4.0 * MK5n.WHEEL_RADIUS_METERS * MK5n.WHEEL_RADIUS_METERS);
  private static final Voltage kSteerFrictionVoltage = Volts.of(0.2);
  private static final Voltage kDriveFrictionVoltage = Volts.of(0.2);

  public static final SwerveDrivetrainConstants DrivetrainConstants =
      new SwerveDrivetrainConstants()
          .withNetwork(kCANBus)
          .withPigeon2Id(kPigeonId)
          .withPigeon2Configs(pigeonConfigs);

  private static final SwerveModuleConstantsFactory<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      ConstantCreator =
          new SwerveModuleConstantsFactory<
                  TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
              .withDriveMotorGearRatio(kDriveGearRatio)
              .withSteerMotorGearRatio(kSteerGearRatio)
              .withCouplingGearRatio(kCoupleRatio)
              .withWheelRadius(kWheelRadius)
              .withSteerMotorGains(steerGains)
              .withDriveMotorGains(driveGains)
              .withSteerMotorClosedLoopOutput(kSteerClosedLoopOutput)
              .withDriveMotorClosedLoopOutput(kDriveClosedLoopOutput)
              .withSlipCurrent(kSlipCurrent)
              .withSpeedAt12Volts(kSpeedAt12Volts)
              .withDriveMotorType(kDriveMotorType)
              .withSteerMotorType(kSteerMotorType)
              .withFeedbackSource(kSteerFeedbackType)
              .withDriveMotorInitialConfigs(driveInitialConfigs)
              .withSteerMotorInitialConfigs(steerInitialConfigs)
              .withEncoderInitialConfigs(encoderInitialConfigs)
              .withSteerInertia(kSteerInertia)
              .withDriveInertia(kDriveInertia)
              .withSteerFrictionVoltage(kSteerFrictionVoltage)
              .withDriveFrictionVoltage(kDriveFrictionVoltage);

  // CAN IDs follow drive, steer, encoder per module in FL, FR, BL, BR order.

  // Front Left
  private static final int kFrontLeftDriveMotorId = 1;
  private static final int kFrontLeftSteerMotorId = 2;
  private static final int kFrontLeftEncoderId = 3;
  private static final Angle kFrontLeftEncoderOffset = Rotations.of(0.0);
  private static final boolean kFrontLeftSteerMotorInverted = true;
  private static final boolean kFrontLeftEncoderInverted = false;

  private static final Distance kFrontLeftXPos = Inches.of(11.5);
  private static final Distance kFrontLeftYPos = Inches.of(11.5);

  // Front Right
  private static final int kFrontRightDriveMotorId = 4;
  private static final int kFrontRightSteerMotorId = 5;
  private static final int kFrontRightEncoderId = 6;
  private static final Angle kFrontRightEncoderOffset = Rotations.of(0.0);
  private static final boolean kFrontRightSteerMotorInverted = true;
  private static final boolean kFrontRightEncoderInverted = false;

  private static final Distance kFrontRightXPos = Inches.of(11.5);
  private static final Distance kFrontRightYPos = Inches.of(-11.5);

  // Back Left
  private static final int kBackLeftDriveMotorId = 7;
  private static final int kBackLeftSteerMotorId = 8;
  private static final int kBackLeftEncoderId = 9;
  private static final Angle kBackLeftEncoderOffset = Rotations.of(0.0);
  private static final boolean kBackLeftSteerMotorInverted = true;
  private static final boolean kBackLeftEncoderInverted = false;

  private static final Distance kBackLeftXPos = Inches.of(-11.5);
  private static final Distance kBackLeftYPos = Inches.of(11.5);

  // Back Right
  private static final int kBackRightDriveMotorId = 10;
  private static final int kBackRightSteerMotorId = 11;
  private static final int kBackRightEncoderId = 12;
  private static final Angle kBackRightEncoderOffset = Rotations.of(0.0);
  private static final boolean kBackRightSteerMotorInverted = true;
  private static final boolean kBackRightEncoderInverted = false;

  private static final Distance kBackRightXPos = Inches.of(-11.5);
  private static final Distance kBackRightYPos = Inches.of(-11.5);

  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      FrontLeft =
          ConstantCreator.createModuleConstants(
              kFrontLeftSteerMotorId,
              kFrontLeftDriveMotorId,
              kFrontLeftEncoderId,
              kFrontLeftEncoderOffset,
              kFrontLeftXPos,
              kFrontLeftYPos,
              kInvertLeftSide,
              kFrontLeftSteerMotorInverted,
              kFrontLeftEncoderInverted);
  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      FrontRight =
          ConstantCreator.createModuleConstants(
              kFrontRightSteerMotorId,
              kFrontRightDriveMotorId,
              kFrontRightEncoderId,
              kFrontRightEncoderOffset,
              kFrontRightXPos,
              kFrontRightYPos,
              kInvertRightSide,
              kFrontRightSteerMotorInverted,
              kFrontRightEncoderInverted);
  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      BackLeft =
          ConstantCreator.createModuleConstants(
              kBackLeftSteerMotorId,
              kBackLeftDriveMotorId,
              kBackLeftEncoderId,
              kBackLeftEncoderOffset,
              kBackLeftXPos,
              kBackLeftYPos,
              kInvertLeftSide,
              kBackLeftSteerMotorInverted,
              kBackLeftEncoderInverted);
  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      BackRight =
          ConstantCreator.createModuleConstants(
              kBackRightSteerMotorId,
              kBackRightDriveMotorId,
              kBackRightEncoderId,
              kBackRightEncoderOffset,
              kBackRightXPos,
              kBackRightYPos,
              kInvertRightSide,
              kBackRightSteerMotorInverted,
              kBackRightEncoderInverted);
}
