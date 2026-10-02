// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package first.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;

/**
 * The robot program's top level. Main.java (one folder up, in {@code first/}) creates it when the
 * program starts, and WPILib then calls its methods: robotPeriodic every 20 ms, and an init method
 * each time the mode changes (disabled, autonomous, teleop).
 *
 * <p>LoggedRobot is AdvantageKit's version of WPILib's robot base class; it adds logging. The
 * constructor below is mostly logging setup that rarely needs changing. The interesting part is
 * robotPeriodic.
 */
public class Robot extends LoggedRobot {
  private final Scheduler scheduler = Scheduler.getDefault();
  private final RobotContainer robotContainer;
  private Command autonomousCommand;

  public Robot() {
    // Record which version of the code is running, so a log always says what made it.
    // BuildConstants.java is written by the build from git (see gversion in build.gradle), and
    // .gitignore keeps it out of git.
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    Logger.recordMetadata(
        "GitDirty",
        switch (BuildConstants.DIRTY) {
          case 0 -> "All changes committed";
          case 1 -> "Uncommitted changes";
          default -> "Unknown";
        });

    // Set up data receivers & replay source
    switch (Constants.CURRENT_MODE) {
      case REAL:
      case SIM:
        // Log to a file (a USB stick on the robot, "logs/" in the simulator) and live to
        // NetworkTables, so AdvantageScope can watch while it runs.
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // Start AdvantageKit logger
    Logger.start();

    // After Logger.start() so IO constructed here is recorded from the first cycle.
    robotContainer = new RobotContainer();
  }

  /**
   * LoggedRobot calls this every 20 ms in every mode, after the mode-specific init (if the mode
   * just changed). AdvantageKit records the inputs read here, which is what makes replay possible.
   */
  @Override
  public void robotPeriodic() {
    // Inputs first, so triggers and commands act on this cycle's data.
    robotContainer.periodic();
    // The scheduler runs every active command for one loop.
    scheduler.run();
  }

  @Override
  public void disabledInit() {
    cancelAutonomous();
  }

  // Empty overrides here and below silence WPILib's "override me" console messages. Everything
  // the robot does each loop runs from robotPeriodic through commands, not per-mode methods.
  @Override
  public void disabledPeriodic() {}

  @Override
  public void simulationPeriodic() {}

  @Override
  public void autonomousInit() {
    robotContainer.resetToStartPose();
    autonomousCommand = robotContainer.getAutonomousCommand();
    // Null if nothing is selected on the dashboard.
    if (autonomousCommand != null) {
      scheduler.schedule(autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    cancelAutonomous();
  }

  /**
   * Stops the auto routine when autonomous ends. Some routines (the characterization ones, which
   * measure the robot) run until stopped and print their results when they are, so this is also
   * what makes them print.
   */
  private void cancelAutonomous() {
    if (autonomousCommand != null) {
      scheduler.cancel(autonomousCommand);
      autonomousCommand = null;
    }
  }

  @Override
  public void teleopPeriodic() {}

  // Utility is a Driver Station mode new in 2027, for running things outside a match. This robot
  // doesn't use it.
  @Override
  public void utilityInit() {}

  @Override
  public void utilityPeriodic() {}
}
