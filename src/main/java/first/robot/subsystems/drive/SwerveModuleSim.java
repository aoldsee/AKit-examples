package first.robot.subsystems.drive;

import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import first.robot.Constants;
import first.robot.sim.PhoenixSimUtil;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.system.Models;
import org.wpilib.math.util.Units;
import org.wpilib.simulation.DCMotorSim;

/**
 * Physics for one swerve module: how hard its wheel pushes on the ground, and how its steering
 * turns. {@link SwerveDriveSim} adds up the four pushes and moves the robot.
 *
 * <p>The wheel's push comes from the same characterization constants the gains use (kS, kV, kA in
 * DriveConstants), so characterizing the real robot updates the sim too. Steering is simpler: each
 * module's steer motor turns only the module, so it's modeled on its own, as a motor plus inertia.
 *
 * <p>Also models the MK5n bevel coupling (steering also turns the drive motor), so
 * ModuleIOTalonFX's compensation is exercised.
 */
class SwerveModuleSim {
  // How stiffly the tread grips the carpet. The tread pushes against any difference between how
  // the wheel moves and how the ground under it moves, harder the bigger the difference. Real
  // tread is very stiff; this is softened so any slide dies out in about 20 ms, which looks rigid
  // but keeps the math stable at the sim's 4 ms step. Newtons per (m/s) of sliding.
  private static final double GRIP_TIME_CONSTANT_SECS = 0.02;
  private static final double GRIP_STIFFNESS =
      Constants.ROBOT_MASS_KG / 4.0 / GRIP_TIME_CONSTANT_SECS;

  // The most any one wheel can push before it slips: grip (coefficient of friction) times the
  // weight on that wheel.
  private static final double MAX_WHEEL_FORCE =
      DriveConstants.WHEEL_COF * Constants.ROBOT_MASS_KG * 9.81 / 4.0;

  // Real friction grabs hard at zero speed. Modeling that exactly makes a sim jitter, so here it
  // fades in over the first centimeter per second instead. Below that, a push smaller than kS can
  // creep the robot very slowly. m/s.
  private static final double FRICTION_FADE_SPEED = 0.01;

  // Characterization constants converted from wheel rotations to meters along the ground.
  private static final double METERS_PER_WHEEL_ROTATION =
      2 * Math.PI * DriveConstants.WHEEL_RADIUS_METERS;
  private static final double VOLTS_PER_METER_PER_SEC =
      DriveConstants.DRIVE_KV / METERS_PER_WHEEL_ROTATION;

  // Newtons of push per volt. kA says how many volts it takes to accelerate this wheel's quarter
  // of the robot; turned around, that's how hard a volt pushes. (Worked out from the motor spec,
  // it's torque per amp * gear ratio / (resistance * wheel radius).)
  private static final double NEWTONS_PER_VOLT =
      Constants.ROBOT_MASS_KG / 4.0 * METERS_PER_WHEEL_ROTATION / DriveConstants.DRIVE_KA;

  private final TalonFXSimState driveSim;
  private final TalonFXSimState steerSim;
  private final CANcoderSimState encoderSim;
  private final DCMotorSim steerMotor;
  private final Translation2d location;

  // The wheel's own speed along the ground, m/s, and how far it has turned, radians. Equal to the
  // ground's speed unless the wheel is slipping.
  private double wheelSpeed = 0.0;
  private double wheelAngleRad = 0.0;
  private double supplyCurrentAmps = 0.0;

  SwerveModuleSim(
      DriveConstants.ModuleConfig module,
      TalonFXSimState driveSim,
      TalonFXSimState steerSim,
      CANcoderSimState encoderSim) {
    this.driveSim = driveSim;
    this.steerSim = steerSim;
    this.encoderSim = encoderSim;
    location = module.location();

    // Same config ModuleIOTalonFX set the real devices up from, so directions always agree.
    driveSim.Orientation = PhoenixSimUtil.orientation(module.driveMotorDirection());
    steerSim.Orientation = PhoenixSimUtil.orientation(module.steerMotorDirection());
    encoderSim.Orientation = PhoenixSimUtil.orientation(module.encoderDirection());
    encoderSim.SensorOffset = module.encoderOffsetRotations();
    driveSim.setMotorType(MotorType.KrakenX60);
    steerSim.setMotorType(MotorType.KrakenX44);

    // Built from the steer's kV and kA. The plant works in radians, so the per-rotation values
    // are divided by 2 pi. DCMotorSim wants a plant that tracks position as well as speed;
    // despite the name, the arm model is exactly that, with no gravity added.
    steerMotor =
        new DCMotorSim(
            Models.singleJointedArmFromSysId(
                DriveConstants.STEER_KV / (2 * Math.PI), DriveConstants.STEER_KA / (2 * Math.PI)),
            DriveConstants.STEER_MOTOR.toDCMotor(1));
  }

  Translation2d getLocation() {
    return location;
  }

  /** Which way the wheel points, relative to the robot. */
  Rotation2d getAngle() {
    return Rotation2d.fromRadians(steerMotor.getAngularPosition());
  }

  /** Battery current both motors drew during the last update. */
  double getSupplyCurrentAmps() {
    return supplyCurrentAmps;
  }

  /** Step 1 of each update: give the devices battery power, and turn the module. */
  void updateSteering(double dtSeconds, double batteryVolts) {
    driveSim.setSupplyVoltage(batteryVolts);
    steerSim.setSupplyVoltage(batteryVolts);
    encoderSim.setSupplyVoltage(batteryVolts);

    double volts = steerSim.getMotorVoltage();
    double before = steerMotor.getAngularVelocity();
    steerMotor.setInputVoltage(
        PhoenixSimUtil.applyFriction(volts, before, DriveConstants.STEER_KS));
    steerMotor.update(dtSeconds);
    if (PhoenixSimUtil.stoppedByFriction(
        before, steerMotor.getAngularVelocity(), volts, DriveConstants.STEER_KS)) {
      steerMotor.setAngularVelocity(0.0);
    }
  }

  /**
   * Step 2: how hard this wheel pushes on the robot, in newtons, robot-relative.
   *
   * @param moduleVelocity how fast this module is moving over the ground, in the robot's frame, m/s
   *     (the robot's own motion, including spin, seen from this spot)
   */
  Translation2d push(Translation2d moduleVelocity) {
    // Split the module's motion into rolling (along the wheel) and sliding (across it).
    var along = new Translation2d(1.0, getAngle());
    var across = new Translation2d(1.0, getAngle().plus(Rotation2d.CCW_90DEG));
    double rolling = moduleVelocity.getX() * along.getX() + moduleVelocity.getY() * along.getY();
    double sliding = moduleVelocity.getX() * across.getX() + moduleVelocity.getY() * across.getY();

    // The bevel gear turns the drive motor whenever the module steers, even with the wheel still.
    // The motor's back-EMF and friction follow how fast the motor spins, so count that extra
    // turning as wheel speed. ModuleIOTalonFX adds voltage for exactly this, and here that voltage
    // is used up spinning the motor instead of pushing the robot.
    double coupling =
        steerMotor.getAngularVelocity()
            * DriveConstants.COUPLING_GEAR_RATIO
            / DriveConstants.DRIVE_GEAR_RATIO
            * DriveConstants.WHEEL_RADIUS_METERS;
    double motorSpeed = rolling + coupling;

    // The characterization equation, volts = kS + kV * speed + kA * acceleration, solved for the
    // push. Whatever voltage is left after friction and back-EMF accelerates the robot.
    double volts = driveSim.getMotorVoltage();
    double friction =
        DriveConstants.DRIVE_KS * Math.clamp(motorSpeed / FRICTION_FADE_SPEED, -1.0, 1.0);
    double drivePush = NEWTONS_PER_VOLT * (volts - friction - VOLTS_PER_METER_PER_SEC * motorSpeed);

    // The tread resists sliding sideways.
    double sidePush = -GRIP_STIFFNESS * sliding;

    // Traction: the total push can't beat grip. If it would, the wheel slips and the push is
    // capped at what grip allows.
    double total = Math.hypot(drivePush, sidePush);
    if (total > MAX_WHEEL_FORCE) {
      drivePush *= MAX_WHEEL_FORCE / total;
      sidePush *= MAX_WHEEL_FORCE / total;
      // A slipping wheel spins faster (or slower) than the ground: as fast as the motor can turn
      // while only pushing that hard.
      wheelSpeed =
          (volts - friction - drivePush / NEWTONS_PER_VOLT) / VOLTS_PER_METER_PER_SEC - coupling;
    } else {
      wheelSpeed = rolling;
    }
    return along.times(drivePush).plus(across.times(sidePush));
  }

  /** Step 3: move the wheel and write every simulated sensor. */
  void updateSensors(double dtSeconds) {
    double wheelRadPerSec = wheelSpeed / DriveConstants.WHEEL_RADIUS_METERS;
    wheelAngleRad += wheelRadPerSec * dtSeconds;

    // Sim state setters take rotations, the motor model gives radians. The drive rotor turns with
    // the wheel and also with the module through the bevel.
    driveSim.setRawRotorPosition(
        Units.radiansToRotations(
            wheelAngleRad * DriveConstants.DRIVE_GEAR_RATIO
                + steerMotor.getAngularPosition() * DriveConstants.COUPLING_GEAR_RATIO));
    driveSim.setRotorVelocity(
        Units.radiansToRotations(
            wheelRadPerSec * DriveConstants.DRIVE_GEAR_RATIO
                + steerMotor.getAngularVelocity() * DriveConstants.COUPLING_GEAR_RATIO));

    steerSim.setRawRotorPosition(
        Units.radiansToRotations(
            steerMotor.getAngularPosition() * DriveConstants.STEER_GEAR_RATIO));
    steerSim.setRotorVelocity(
        Units.radiansToRotations(
            steerMotor.getAngularVelocity() * DriveConstants.STEER_GEAR_RATIO));
    steerSim.setRotorAcceleration(
        Units.radiansToRotations(
            steerMotor.getAngularAcceleration() * DriveConstants.STEER_GEAR_RATIO));

    // The CANcoder sits on the module, after the steer reduction.
    encoderSim.setRawPosition(Units.radiansToRotations(steerMotor.getAngularPosition()));
    encoderSim.setVelocity(Units.radiansToRotations(steerMotor.getAngularVelocity()));

    // Phoenix's sim works out each Talon's battery current from its output and motor speed, using
    // its own model of the motor.
    supplyCurrentAmps = driveSim.getSupplyCurrent() + steerSim.getSupplyCurrent();
  }
}
