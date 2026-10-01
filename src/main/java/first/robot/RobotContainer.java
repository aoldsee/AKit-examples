package first.robot;

import com.ctre.phoenix6.SignalLogger;
import first.robot.commands.AlignTargets;
import first.robot.commands.DriveCommands;
import first.robot.commands.DriveCommands.SysIdDirection;
import first.robot.commands.DriveSystemsCheck;
import first.robot.sim.SimWorld;
import first.robot.subsystems.arm.Arm;
import first.robot.subsystems.arm.ArmConstants;
import first.robot.subsystems.arm.ArmIO;
import first.robot.subsystems.arm.ArmIOTalonFX;
import first.robot.subsystems.arm.ArmIOTalonFXSim;
import first.robot.subsystems.arm.ArmSim;
import first.robot.subsystems.drive.Drive;
import first.robot.subsystems.drive.DriveConstants;
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
import first.robot.util.Battery;
import first.robot.util.FieldGeometry;
import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkChooser;
import org.wpilib.command3.Command;
import org.wpilib.command3.button.CommandGamepad;
import org.wpilib.driverstation.GenericHID.RumbleType;
import org.wpilib.driverstation.RobotState;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.units.Units;

/** Builds the mechanisms with the IO for the current mode, plus the sim models in SIM. */
public class RobotContainer {
  private final Drive drive;
  private final Arm arm;
  private final Vision vision;
  private final Intake intake;

  // Only used in SIM. The first two are null and the list is empty otherwise.
  private final SimWorld simWorld;
  private final SwerveDriveSim driveSim;
  private final List<LimelightSim> limelightSims = new ArrayList<>();

  private final CommandGamepad driver = new CommandGamepad(0);
  private final LoggedNetworkChooser<Command> autoChooser =
      new LoggedNetworkChooser<>("Auto Choices");
  // Where the robot is placed before the match. Odometry starts here (see periodic).
  private final LoggedNetworkChooser<Integer> startChooser =
      new LoggedNetworkChooser<>("Starting Position");
  // The last starting pose odometry was reset to, so it's only reset again when it changes.
  private Pose2d lastStartPose = null;

  public RobotContainer() {

    // The one place the three modes differ: which IO each mechanism gets.
    switch (Constants.currentMode) {
      case REAL -> {
        simWorld = null;
        driveSim = null;
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(DriveConstants.MODULE_CONSTANTS[0]),
                new ModuleIOTalonFX(DriveConstants.MODULE_CONSTANTS[1]),
                new ModuleIOTalonFX(DriveConstants.MODULE_CONSTANTS[2]),
                new ModuleIOTalonFX(DriveConstants.MODULE_CONSTANTS[3]));
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
                new ModuleIOTalonFXSim(DriveConstants.MODULE_CONSTANTS[0], driveSim),
                new ModuleIOTalonFXSim(DriveConstants.MODULE_CONSTANTS[1], driveSim),
                new ModuleIOTalonFXSim(DriveConstants.MODULE_CONSTANTS[2], driveSim),
                new ModuleIOTalonFXSim(DriveConstants.MODULE_CONSTANTS[3], driveSim));

        var armSim = new ArmSim();
        simWorld.add("Arm", armSim);
        arm = new Arm(new ArmIOTalonFXSim(armSim));

        // Starts holding a piece, like a robot set up for auto.
        var intakeSim = new IntakeSim(true);
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

    configureBindings();
    configureAutos();
    for (int station = 1; station <= FieldGeometry.STATION_COUNT; station++) {
      if (station == 1) {
        startChooser.addDefault("Station 1", station);
      } else {
        startChooser.add("Station " + station, station);
      }
    }
  }

  /**
   * Bindings use CommandGamepad's positional names, so they work with any gamepad the Driver
   * Station recognizes. The DS maps every controller onto the same layout:
   *
   * <pre>
   * CommandGamepad   Xbox      DualSense
   * faceDown         A         Cross
   * faceRight        B         Circle
   * faceLeft         X         Square
   * faceUp           Y         Triangle
   * left/rightBumper LB/RB     L1/R1
   * left/rightTrigger LT/RT    L2/R2
   * back / start     View/Menu Create/Options
   * </pre>
   *
   * CommandXboxController and CommandDualSenseController read these same buttons under brand names.
   * They add no buttons CommandGamepad lacks (it has the touchpad and misc buttons too), so
   * choosing one is only about which names read better.
   */
  private void configureBindings() {
    // An interlock: one mechanism's state limiting another. Away from its hard stops, the arm is
    // held up only by its motor, so a hard start or stop swings it. While it's raised, every
    // joystick drive command eases off.
    DoubleSupplier driveAccelCap =
        () -> arm.isNearHardStop() ? Double.POSITIVE_INFINITY : DriveConstants.ARM_RAISED_MAX_ACCEL;

    // Stick axes are +down and +right, field axes are +away and +left, hence the negations.
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driver.getLeftY(),
            () -> -driver.getLeftX(),
            () -> -driver.getRightX(),
            driveAccelCap));

    // Hold A to keep the robot facing downfield while still translating with the left stick.
    driver
        .faceDown()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -driver.getLeftY(),
                () -> -driver.getLeftX(),
                () -> Rotation2d.ZERO,
                driveAccelCap));

    // Hold to keep facing our hub while still translating with the left stick, for aiming on the
    // move. The heading is worked out fresh every loop, so it tracks as the robot moves.
    driver
        .leftBumper()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -driver.getLeftY(),
                () -> -driver.getLeftX(),
                () ->
                    FieldGeometry.headingToward(
                        drive.getPose(), FieldGeometry.flipIfRed(FieldGeometry.BLUE_HUB_CENTER)),
                driveAccelCap));

    // Hold to line up in front of the nearest AprilTag. Letting go cancels it, and the sticks take
    // over again.
    driver.rightBumper().whileTrue(DriveCommands.driveToPose(drive, this::nearestAlignTarget));
    // Buzz the controller when the robot gets there, so the driver doesn't have to look at a
    // dashboard. A Trigger can watch any condition, not just a button.
    driver
        .rightBumper()
        .and(
            () ->
                AlignTargets.isAligned(
                    drive.getPose(), drive.getChassisVelocities(), nearestAlignTarget()))
        .onTrue(rumble(0.3));

    intake.setDefaultCommand(intake.hold());
    driver.leftTrigger().whileTrue(intake.intake());
    driver.rightTrigger().whileTrue(intake.eject());

    arm.setDefaultCommand(arm.hold());
    driver.dpadUp().onTrue(arm.goTo(ArmConstants.VERTICAL_RAD));
    driver.dpadRight().onTrue(arm.goTo(ArmConstants.HORIZONTAL_RAD));
    driver.dpadDown().onTrue(arm.goTo(ArmConstants.STOWED_RAD));

    driver.faceLeft().onTrue(drive.run(coroutine -> drive.stopWithX()).named("Drive.XLock"));

    // Tell odometry the robot is facing away from the driver, keeping its position.
    driver
        .faceRight()
        .onTrue(
            drive
                .run(
                    coroutine -> {
                      var heading = FieldGeometry.isRed() ? Rotation2d.k180deg : Rotation2d.ZERO;
                      resetPose(new Pose2d(drive.getPose().getTranslation(), heading));
                    })
                .named("Drive.ResetHeading"));
  }

  private void configureAutos() {
    // PathPlanner autos go here once PathPlannerLib supports Commands v3. Its AutoBuilder builds
    // v2 commands, and the v2 and v3 vendordeps can't be installed together.
    autoChooser.addDefault("None", drive.run(coroutine -> drive.stop()).named("Auto.None"));
    autoChooser.add(
        "Drive Forward 2 m",
        DriveCommands.driveFor(drive, new ChassisVelocities(1.0, 0.0, 0.0), 2.0));

    // Sequencing in a coroutine reads top to bottom: each await finishes before the next line.
    autoChooser.add(
        "Arm Up, Pause, Stow",
        Command.noRequirements(
                coroutine -> {
                  coroutine.await(arm.goTo(ArmConstants.VERTICAL_RAD));
                  coroutine.wait(Units.Seconds.of(1.0));
                  coroutine.await(arm.goTo(ArmConstants.STOWED_RAD));
                })
            .named("Auto.ArmDemo"));

    autoChooser.add(
        "Align to Nearest Tag", DriveCommands.driveToPose(drive, this::nearestAlignTarget));

    // A scoring-style routine: drive into position and raise the arm at the same time, then wait
    // until both have arrived before doing anything else.
    autoChooser.add(
        "Align to Nearest Tag While Raising Arm",
        Command.noRequirements(
                coroutine ->
                    coroutine.awaitAll(
                        DriveCommands.driveToPose(drive, this::nearestAlignTarget),
                        arm.goTo(ArmConstants.HORIZONTAL_RAD)))
            .named("Auto.AlignAndRaiseArm"));

    // awaitAll runs both at once and finishes when both have. They don't conflict because each
    // requires a different mechanism.
    autoChooser.add(
        "Drive Forward While Raising Arm",
        Command.noRequirements(
                coroutine ->
                    coroutine.awaitAll(
                        DriveCommands.driveFor(drive, new ChassisVelocities(1.0, 0.0, 0.0), 2.0),
                        arm.goTo(ArmConstants.HORIZONTAL_RAD)))
            .named("Auto.DriveAndArm"));
    // A full scoring routine. Each await finishes before the next line starts.
    autoChooser.add(
        "Score Preload at Nearest Tag",
        Command.noRequirements(
                coroutine -> {
                  coroutine.awaitAll(
                      DriveCommands.driveToPose(drive, this::nearestAlignTarget),
                      arm.goTo(ArmConstants.HORIZONTAL_RAD));
                  coroutine.await(intake.eject());
                  coroutine.await(arm.goTo(ArmConstants.STOWED_RAD));
                })
            .named("Auto.ScorePreload"));
    autoChooser.add("Drive Systems Check", DriveSystemsCheck.create(drive));
    autoChooser.add("Drive FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.add(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.add(
        "Drive SysId (Quasistatic Forward)",
        DriveCommands.sysIdQuasistatic(drive, SysIdDirection.FORWARD));
    autoChooser.add(
        "Drive SysId (Quasistatic Reverse)",
        DriveCommands.sysIdQuasistatic(drive, SysIdDirection.REVERSE));
    autoChooser.add(
        "Drive SysId (Dynamic Forward)", DriveCommands.sysIdDynamic(drive, SysIdDirection.FORWARD));
    autoChooser.add(
        "Drive SysId (Dynamic Reverse)", DriveCommands.sysIdDynamic(drive, SysIdDirection.REVERSE));
  }

  /** Where to line up: in front of the nearest reachable tag, or stay put if there isn't one. */
  private Pose2d nearestAlignTarget() {
    return AlignTargets.nearestTag(drive.getPose()).orElse(drive.getPose());
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

  /** Rumbles both sides of the driver's controller for a moment. */
  private Command rumble(double seconds) {
    return Command.noRequirements(
            coroutine -> {
              setRumble(1.0);
              coroutine.wait(Units.Seconds.of(seconds));
              setRumble(0.0);
            })
        // Without this, canceling mid-rumble would leave the controller buzzing.
        .whenCanceled(() -> setRumble(0.0))
        .named("Driver.Rumble");
  }

  private void setRumble(double strength) {
    driver.getHID().setRumble(RumbleType.LEFT_RUMBLE, strength);
    driver.getHID().setRumble(RumbleType.RIGHT_RUMBLE, strength);
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
