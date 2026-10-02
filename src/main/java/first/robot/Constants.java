// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package first.robot;

import org.wpilib.framework.RobotBase;
import org.wpilib.math.util.Units;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a SystemCore. Change the value of "SIM_MODE" to switch between "sim" (physics sim) and
 * "replay" (log replay from a file).
 */
public final class Constants {
  public static final Mode SIM_MODE = Mode.SIM;
  public static final Mode CURRENT_MODE = RobotBase.isReal() ? Mode.REAL : SIM_MODE;

  /**
   * When true, gains wrapped in TunableNumber can be edited live from the dashboard under
   * "/Tuning". Turn it off for competition so nobody can change a gain mid-match by accident.
   */
  public static final boolean TUNING_MODE = true;

  /**
   * Everything that moves when the robot drives: frame, mechanisms, bumpers, and battery. 140 lb is
   * about a full-weight competition robot. Weigh the real one; acceleration, current draw, and
   * brownouts in the simulator all scale with it.
   */
  public static final double ROBOT_MASS_KG = Units.lbsToKilograms(140.0);

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
}
