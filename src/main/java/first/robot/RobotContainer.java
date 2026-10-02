package first.robot;

import com.ctre.phoenix6.SignalLogger;
import first.robot.autos.Autos;
import first.robot.field.FieldGeometry;
import first.robot.sim.Battery;
import first.robot.sim.SimWorld;
import first.robot.subsystems.arm.Arm;
import first.robot.subsystems.arm.ArmIO;
import first.robot.subsystems.arm.ArmIOTalonFX;
import first.robot.subsystems.arm.ArmIOTalonFXSim;
import first.robot.subsystems.arm.ArmSim;
import first.robot.subsystems.drive.Drive;
import first.robot.subsystems.drive.DriveCharacterization;
import first.robot.subsystems.drive.DriveCharacterization.SysIdDirection;
import first.robot.subsystems.drive.DriveCommands;
import first.robot.subsystems.drive.DriveConstants;
import first.robot.subsystems.drive.DriveSystemsCheck;
import first.robot.subsystems.drive.GyroIO;
import first.robot.subsystems.drive.GyroIOPigeon2;
import first.robot.subsystems.drive.GyroIOPigeon2Sim;
import first.robot.subsystems.drive.ModuleIO;
import first.robot.subsystems.drive.ModuleIOTalonFX;
import first.robot.subsystems.drive.ModuleIOTalonFXSim;
import first.robot.subsystems.drive.SwerveDriveSim;
import first.robot.subsystems.intake.Intake;
import first.robot.subsystems.intake.IntakeIO;
import first.robot.subsystems.intake.IntakeIOTalonFX;
import first.robot.subsystems.intake.IntakeIOTalonFXSim;
import first.robot.subsystems.intake.IntakeSim;
import first.robot.subsystems.vision.LimelightSim;
import first.robot.subsystems.vision.Vision;
import first.robot.subsystems.vision.VisionConstants;
import first.robot.subsystems.vision.VisionIO;
import first.robot.subsystems.vision.VisionIOLimelight;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkChooser;
import org.wpilib.command3.Command;
import org.wpilib.command3.Trigger;
import org.wpilib.driverstation.RobotState;
import org.wpilib.math.geometry.Pose2d;

/**
 * Builds the robot and connects it to the controller and dashboard.
 *
 * <ul>
 *   <li>The constructor picks the IO for each mechanism, depending on whether this is a real robot,
 *       the simulator, or a replay. Skim it; the three cases build the same things.
 *   <li>{@link Controls} (a separate file) connects the gamepad's buttons to commands.
 *   <li>{@link #configureAutos()} lists the autonomous routines for the dashboard.
 *   <li>{@link #periodic()} updates every mechanism once per loop.
 * </ul>
 */
public class RobotContainer {
  private final Drive drive;
  private final Arm arm;
  private final Vision vision;
  private final Intake intake;

  // Only used in SIM. The first two are null and the list is empty otherwise.
  private final SimWorld simWorld;
  private final SwerveDriveSim driveSim;
  private final List<LimelightSim> limelightSims = new ArrayList<>();

  private final LoggedNetworkChooser<Command> autoChooser =
      new LoggedNetworkChooser<>("Auto Choices");
  // Where the robot is placed before the match. Odometry starts here (see periodic).
  private final LoggedNetworkChooser<Integer> startChooser =
      new LoggedNetworkChooser<>("Starting Position");
  // The last starting pose odometry was reset to, so it's only reset again when it changes.
  private Pose2d lastStartPose = null;

  public RobotContainer() {

    // The one place the three modes differ: which IO each mechanism gets.
    switch (Constants.CURRENT_MODE) {
      case REAL -> {
        simWorld = null;
        driveSim = null;
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(DriveConstants.MODULES[0]),
                new ModuleIOTalonFX(DriveConstants.MODULES[1]),
                new ModuleIOTalonFX(DriveConstants.MODULES[2]),
                new ModuleIOTalonFX(DriveConstants.MODULES[3]));
        arm = new Arm(new ArmIOTalonFX());
        intake = new Intake(new IntakeIOTalonFX());
        vision =
            new Vision(
                drive::addVisionMeasurement,
                this::yawVelocity,
                new VisionIOLimelight(
                    VisionConstants.CAMERA_NAMES[0],
                    VisionConstants.ROBOT_TO_CAMERA[0],
                    drive::getRotation),
                new VisionIOLimelight(
                    VisionConstants.CAMERA_NAMES[1],
                    VisionConstants.ROBOT_TO_CAMERA[1],
                    drive::getRotation));
      }
      case SIM -> {
        // Same device code as REAL, talking to Phoenix's simulated devices. Each mechanism's sim
        // model plays the hardware behind them, and simWorld steps all the models. It starts
        // last, once every device has registered.
        // Phoenix's own hoot logs are worth keeping on a real robot for Tuner X, but AdvantageKit
        // already records everything, and in sim they'd pile up a new file per run.
        SignalLogger.enableAutoLogging(false);
        simWorld = new SimWorld(new Battery());
        driveSim = new SwerveDriveSim();
        simWorld.add("Drive", driveSim);
        drive =
            new Drive(
                new GyroIOPigeon2Sim(driveSim),
                new ModuleIOTalonFXSim(DriveConstants.MODULES[0], driveSim),
                new ModuleIOTalonFXSim(DriveConstants.MODULES[1], driveSim),
                new ModuleIOTalonFXSim(DriveConstants.MODULES[2], driveSim),
                new ModuleIOTalonFXSim(DriveConstants.MODULES[3], driveSim));

        var armSim = new ArmSim();
        simWorld.add("Arm", armSim);
        arm = new Arm(new ArmIOTalonFXSim(armSim));

        // Starts holding a piece, like a robot set up for auto.
        var intakeSim = new IntakeSim(IntakeSim.Preload.ONE_PIECE);
        simWorld.add("Intake", intakeSim);
        intake = new Intake(new IntakeIOTalonFXSim(intakeSim));

        // Vision needs no sim IO subclass. Each LimelightSim publishes to NetworkTables exactly
        // like a real Limelight, so the real VisionIOLimelight reads it unchanged.
        for (int i = 0; i < VisionConstants.CAMERA_NAMES.length; i++) {
          var limelightSim =
              new LimelightSim(VisionConstants.CAMERA_NAMES[i], driveSim::getTruePose);
          simWorld.add("Camera" + i, limelightSim);
          limelightSims.add(limelightSim);
        }
        vision =
            new Vision(
                drive::addVisionMeasurement,
                this::yawVelocity,
                new VisionIOLimelight(
                    VisionConstants.CAMERA_NAMES[0],
                    VisionConstants.ROBOT_TO_CAMERA[0],
                    drive::getRotation),
                new VisionIOLimelight(
                    VisionConstants.CAMERA_NAMES[1],
                    VisionConstants.ROBOT_TO_CAMERA[1],
                    drive::getRotation));

        simWorld.start();
      }
      default -> {
        // Replay feeds logged inputs through no-op IO. Constructing Phoenix devices here would
        // start the odometry thread and fight the log.
        // `new ModuleIO() {}` is an anonymous class: an object of a one-off class that implements
        // the interface. Every ModuleIO method has an empty default body, so this one does
        // nothing, and AdvantageKit fills in its inputs from the log instead.
        simWorld = null;
        driveSim = null;
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        arm = new Arm(new ArmIO() {});
        intake = new Intake(new IntakeIO() {});
        vision =
            new Vision(
                drive::addVisionMeasurement,
                this::yawVelocity,
                new VisionIO() {},
                new VisionIO() {});
      }
    }

    Controls.bind(drive, arm, intake);
    configureDashboardButtons();
    configureAutos();
    for (int station = 1; station <= FieldGeometry.STATION_COUNT; station++) {
      if (station == 1) {
        startChooser.addDefault("Station 1", station);
      } else {
        startChooser.add("Station " + station, station);
      }
    }
  }

  /** Dashboard buttons. The gamepad's buttons are in {@link Controls}. */
  private void configureDashboardButtons() {
    // Real-robot calibration; skip this for now. A dashboard button for finding the swerve
    // encoder offsets with the robot disabled (see DriveCharacterization.findEncoderOffsets). It
    // flips back to false once the offsets are printed.
    var findEncoderOffsets = new LoggedNetworkBoolean("/Calibration/FindEncoderOffsets", false);
    new Trigger(findEncoderOffsets::get)
        .onTrue(
            Command.noRequirements(
                    coroutine -> {
                      coroutine.await(DriveCharacterization.findEncoderOffsets(drive));
                      findEncoderOffsets.set(false);
                    })
                .named("Calibration.FindEncoderOffsets"));
  }

  /**
   * The auto chooser's list. The routines themselves are in Autos, or are single-mechanism commands
   * like the characterization routines.
   */
  private void configureAutos() {
    autoChooser.addDefault("None", Autos.none(drive));
    autoChooser.add("Drive Forward 2 m", Autos.driveForward(drive));
    autoChooser.add("Arm Up, Pause, Stow", Autos.armUpPauseStow(arm));
    autoChooser.add("Align to Nearest Tag", DriveCommands.alignToNearestTag(drive));
    autoChooser.add(
        "Align to Nearest Tag While Raising Arm", Autos.alignWhileRaisingArm(drive, arm));
    autoChooser.add(
        "Drive Forward While Raising Arm", Autos.driveForwardWhileRaisingArm(drive, arm));
    autoChooser.add("Score Preload at Nearest Tag", Autos.scorePreload(drive, arm, intake));

    // Pit and tuning routines. Run these with the robot on blocks or with room to move.
    autoChooser.add("Drive Systems Check", DriveSystemsCheck.create(drive));
    autoChooser.add(
        "Drive FF Characterization", DriveCharacterization.feedforwardCharacterization(drive));
    autoChooser.add(
        "Drive Wheel Radius Characterization",
        DriveCharacterization.wheelRadiusCharacterization(drive));
    autoChooser.add(
        "Drive SysId (Quasistatic Forward)",
        DriveCharacterization.sysIdQuasistatic(drive, SysIdDirection.FORWARD));
    autoChooser.add(
        "Drive SysId (Quasistatic Reverse)",
        DriveCharacterization.sysIdQuasistatic(drive, SysIdDirection.REVERSE));
    autoChooser.add(
        "Drive SysId (Dynamic Forward)",
        DriveCharacterization.sysIdDynamic(drive, SysIdDirection.FORWARD));
    autoChooser.add(
        "Drive SysId (Dynamic Reverse)",
        DriveCharacterization.sysIdDynamic(drive, SysIdDirection.REVERSE));
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  /** The selected starting pose, on our alliance's side of the field. */
  public Pose2d getStartPose() {
    return FieldGeometry.flipIfRed(FieldGeometry.startingPose(startChooser.get()));
  }

  /**
   * Resets odometry to the selected starting pose. Every auto assumes it starts there, so the robot
   * calls this when auto begins. In sim, practicing teleop first would otherwise leave the robot
   * somewhere else.
   */
  public void resetToStartPose() {
    lastStartPose = getStartPose();
    resetPose(lastStartPose);
  }

  // Measured from the wheels rather than the gyro, so it still works if the gyro drops out.
  private double yawVelocity() {
    return drive.getChassisVelocities().omega;
  }

  /** Call every loop before the scheduler runs, so commands see this cycle's inputs. */
  public void periodic() {
    drive.periodic();
    // After drive, so vision corrections land on this cycle's odometry.
    vision.periodic();
    arm.periodic();
    intake.periodic();

    // Before a match, the robot sits at its starting position, so odometry starts there too. It's
    // reset whenever the choice or the alliance changes, not every loop, so vision can still
    // correct it in between. In sim this also moves the simulated robot there.
    if (RobotState.isDisabled() && !getStartPose().equals(lastStartPose)) {
      resetToStartPose();
    }
    if (driveSim != null) {
      Logger.recordOutput("Sim/TruePose", driveSim.getTruePose());
      simWorld.logOutputs();
    }
  }

  /** Resets odometry, and in sim moves the simulated robot to match. */
  public void resetPose(Pose2d pose) {
    drive.setPose(pose);
    if (driveSim != null) {
      driveSim.resetTruePose(pose);
      // Otherwise the cameras would report a few frames from where the robot was before.
      limelightSims.forEach(LimelightSim::clearHistory);
    }
  }
}
