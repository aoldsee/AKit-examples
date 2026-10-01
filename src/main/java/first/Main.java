// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package first;

import org.wpilib.framework.RobotBase;

/**
 * Do NOT add any static variables to this class, or any initialization at all. Apart from changing
 * the parameter class in the startRobot call, this file should not need to change.
 */
public final class Main {
  private Main() {}

  /**
   * Main initialization function. Do not perform any initialization here.
   *
   * <p>If the main robot class changes, change the parameter type to match.
   */
  public static void main(String... args) {
    RobotBase.startRobot(first.robot.Robot::new);
  }
}
