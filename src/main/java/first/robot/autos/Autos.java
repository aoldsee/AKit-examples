package first.robot.autos;

import first.robot.subsystems.arm.Arm;
import first.robot.subsystems.arm.ArmConstants;
import first.robot.subsystems.drive.Drive;
import first.robot.subsystems.drive.DriveCommands;
import first.robot.subsystems.intake.Intake;
import org.wpilib.command3.Command;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.units.Units;

/**
 * Autonomous routines built from the mechanisms' own commands.
 *
 * <p>A command that uses one mechanism lives with that mechanism (Arm.goTo, DriveCommands,
 * Intake.eject). Routines that combine several don't belong to any one of them, so they live here.
 * RobotContainer puts them in the auto chooser.
 *
 * <p>Each routine is a coroutine that reads top to bottom: {@code await} runs a command and waits
 * for it to finish, and {@code awaitAll} runs several at once and waits for all of them. Running
 * two commands at once only works when they need different mechanisms.
 *
 * <p>PathPlanner autos would go here too, once PathPlannerLib supports Commands v3. Its AutoBuilder
 * builds v2 commands, and the v2 and v3 vendordeps can't be installed together.
 */
public final class Autos {
  private Autos() {}

  // Forward speed, sideways speed (m/s), and turning speed (rad/s), in the robot's own frame.
  private static final ChassisVelocities FORWARD_1_MPS = new ChassisVelocities(1.0, 0.0, 0.0);

  /** Sits still. The safe default. */
  public static Command none(Drive drive) {
    return drive.run(coroutine -> drive.stop()).named("Auto.None");
  }

  public static Command driveForward(Drive drive) {
    return DriveCommands.driveFor(drive, FORWARD_1_MPS, 2.0);
  }

  /** Sequencing one mechanism: each step finishes before the next starts. */
  public static Command armUpPauseStow(Arm arm) {
    return Command.noRequirements(
            coroutine -> {
              coroutine.await(arm.goTo(ArmConstants.VERTICAL_RAD));
              coroutine.wait(Units.Seconds.of(1.0));
              coroutine.await(arm.goTo(ArmConstants.STOWED_RAD));
            })
        .named("Auto.ArmUpPauseStow");
  }

  /** Two mechanisms at once, finishing when both have. */
  public static Command driveForwardWhileRaisingArm(Drive drive, Arm arm) {
    return Command.noRequirements(
            coroutine ->
                coroutine.awaitAll(
                    DriveCommands.driveFor(drive, FORWARD_1_MPS, 2.0),
                    arm.goTo(ArmConstants.HORIZONTAL_RAD)))
        .named("Auto.DriveForwardWhileRaisingArm");
  }

  /** Drive into position and raise the arm together, then wait until both have arrived. */
  public static Command alignWhileRaisingArm(Drive drive, Arm arm) {
    return Command.noRequirements(
            coroutine ->
                coroutine.awaitAll(
                    DriveCommands.alignToNearestTag(drive), arm.goTo(ArmConstants.HORIZONTAL_RAD)))
        .named("Auto.AlignWhileRaisingArm");
  }

  /** A full scoring routine: line up with the arm raised, eject the preload, then stow. */
  public static Command scorePreload(Drive drive, Arm arm, Intake intake) {
    return Command.noRequirements(
            coroutine -> {
              coroutine.awaitAll(
                  DriveCommands.alignToNearestTag(drive), arm.goTo(ArmConstants.HORIZONTAL_RAD));
              coroutine.await(intake.eject());
              coroutine.await(arm.goTo(ArmConstants.STOWED_RAD));
            })
        .named("Auto.ScorePreload");
  }
}
