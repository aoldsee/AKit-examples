package first.robot;

import first.robot.field.FieldGeometry;
import first.robot.subsystems.arm.Arm;
import first.robot.subsystems.arm.ArmConstants;
import first.robot.subsystems.drive.Drive;
import first.robot.subsystems.drive.DriveCommands;
import first.robot.subsystems.drive.DriveConstants;
import first.robot.subsystems.intake.Intake;
import java.util.function.DoubleSupplier;
import org.wpilib.command3.Command;
import org.wpilib.command3.button.CommandGamepad;
import org.wpilib.driverstation.GenericHID.RumbleType;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.units.Units;

/** Every gamepad button and stick, and what it does. */
public final class Controls {
  private Controls() {}

  /**
   * Connects the driver's gamepad to commands. Bindings use CommandGamepad's positional names, so
   * they work with any gamepad the Driver Station recognizes. The DS maps every controller onto the
   * same layout:
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
  public static void bind(Drive drive, Arm arm, Intake intake) {
    var driver = new CommandGamepad(0);

    // Intake and arm: hold a trigger to run the rollers, tap the D-pad to move the arm. Each
    // mechanism's default command runs whenever no button is using it.
    intake.setDefaultCommand(intake.hold());
    driver.leftTrigger().whileTrue(intake.intake());
    driver.rightTrigger().whileTrue(intake.eject());

    arm.setDefaultCommand(arm.hold());
    driver.dpadUp().onTrue(arm.goTo(ArmConstants.VERTICAL_RAD));
    driver.dpadRight().onTrue(arm.goTo(ArmConstants.HORIZONTAL_RAD));
    driver.dpadDown().onTrue(arm.goTo(ArmConstants.STOWED_RAD));

    // One-shot drive buttons. Each command does its work once and finishes.
    // This command finishes right away, but the X stays. The joystick command takes over and, with
    // the sticks centered, asks for zero speed, and a wheel asked for zero speed keeps pointing
    // where it was (see Drive.stopWithX). Moving the sticks turns the wheels again.
    driver.faceLeft().onTrue(drive.run(coroutine -> drive.stopWithX()).named("Drive.XLock"));
    // Tell odometry the robot is facing away from the driver, keeping its position. Only odometry:
    // the robot hasn't moved, so in sim the simulated robot stays put too.
    driver
        .faceRight()
        .onTrue(
            drive
                .run(
                    coroutine ->
                        drive.setPose(
                            new Pose2d(
                                drive.getPose().getTranslation(), FieldGeometry.downfield())))
                .named("Drive.ResetHeading"));

    // Joystick driving. A pushed-forward stick reads negative y, and pushed-left reads negative x.
    // On the field, +x is away from the driver and +y is to the left, so both get flipped. This
    // flip is only about the stick's sign. Flipping for the red alliance is a separate step, in
    // DriveCommands.driverRelativeHeading.
    //
    // The arm limits the driving. Away from its hard stops, only its motor holds the arm up, so a
    // hard start or stop swings it. So: near a stop, no extra limit (infinity); raised, a gentler
    // acceleration. It's a supplier (a function), not a plain number, so the drive commands call
    // it every loop and see the arm's current state. The stick inputs are lambdas for the same
    // reason.
    DoubleSupplier driveAccelCap =
        () -> arm.isNearHardStop() ? Double.POSITIVE_INFINITY : DriveConstants.ARM_RAISED_MAX_ACCEL;

    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driver.getLeftY(),
            () -> -driver.getLeftX(),
            () -> -driver.getRightX(),
            driveAccelCap));

    // Hold faceDown (A on Xbox) to keep the robot facing downfield while still driving with the
    // left stick.
    driver
        .faceDown()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -driver.getLeftY(),
                () -> -driver.getLeftX(),
                FieldGeometry::downfield,
                driveAccelCap));

    // Hold to keep facing our hub while still driving, for aiming on the move. The heading is
    // worked out fresh every loop, so it tracks as the robot moves.
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

    // Hold to line up in front of the nearest AprilTag, then buzz the controller on arrival so the
    // driver doesn't have to look at a dashboard. A small routine: wait for the alignment to
    // finish (it only finishes once lined up with the tag it picked), then rumble. Letting go
    // cancels it, and the sticks take over again.
    driver
        .rightBumper()
        .whileTrue(
            Command.noRequirements(
                    coroutine -> {
                      coroutine.await(DriveCommands.alignToNearestTag(drive));
                      coroutine.await(rumble(driver, 0.3));
                    })
                .named("Driver.AlignAndRumble"));
  }

  /** Rumbles both sides of the driver's controller for a moment. */
  private static Command rumble(CommandGamepad driver, double seconds) {
    return Command.noRequirements(
            coroutine -> {
              setRumble(driver, 1.0);
              coroutine.wait(Units.Seconds.of(seconds));
              setRumble(driver, 0.0);
            })
        // Without this, canceling mid-rumble would leave the controller buzzing.
        .whenCanceled(() -> setRumble(driver, 0.0))
        .named("Driver.Rumble");
  }

  private static void setRumble(CommandGamepad driver, double strength) {
    driver.getHID().setRumble(RumbleType.LEFT_RUMBLE, strength);
    driver.getHID().setRumble(RumbleType.RIGHT_RUMBLE, strength);
  }
}
