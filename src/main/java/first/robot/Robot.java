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

public class Robot extends LoggedRobot {
  private final Scheduler scheduler = Scheduler.getDefault();
  private final RobotContainer robotContainer;
  private Command autonomousCommand;

  public Robot() {
    // Record metadata
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
    switch (Constants.currentMode) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT and to "logs/" for later viewing or replay
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

  /** Characterization autos run until canceled, so canceling is also what makes them report. */
  private void cancelAutonomous() {
    if (autonomousCommand != null) {
      scheduler.cancel(autonomousCommand);
      autonomousCommand = null;
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void utilityInit() {}

  @Override
  public void utilityPeriodic() {}
}
