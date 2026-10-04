# AKit-Base

A small but complete robot program, using a number of advanced and high-capability techniques and structures. It has a swerve drivetrain, a single-jointed arm with an intake on the end, and AprilTag vision, logs everything with AdvantageKit, and runs in simulation on a laptop, where it can be driven, broken, and examined afterward without a real robot.

It targets WPILib 2027 (alpha 7) on SystemCore (the robot controller replacing the roboRIO), CTRE Phoenix 6 motor controllers, and the new Commands v3 framework. Everything here is a placeholder robot: the hardware values are realistic but not measured from any real machine.

## Getting started

### Requirements

- WPILib 2027 alpha 7 or later. Until 2027 is released, that means an alpha build from [WPILib's releases page](https://github.com/wpilibsuite/allwpilib/releases), not the current season's installer. The [installation guide](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html) covers how to install it. The installer includes the WPILib version of VS Code, AdvantageScope (for looking at logs and the simulated robot), and Elastic (a dashboard).
- A gamepad (Xbox or PlayStation). Without one, the Sim GUI's keyboard joysticks work too.

Get the code (clone it, or download it from GitHub as a zip), then open the folder in WPILib VS Code with **File > Open Folder**.

### Run the simulator

1. In VS Code, open the command palette (Ctrl+Shift+P) and run **WPILib: Simulate Robot Code**.
2. When it asks which extensions to use, tick **Sim GUI**. That window is the simulated Driver Station.
3. In the Sim GUI, drag the controller from **System Joysticks** onto **Joystick[0]**. (No gamepad? Drag **Keyboard 0** instead.)
4. Set the robot state to **Teleoperated** and click **Enable**. The robot now drives.

To see the robot, open AdvantageScope and choose **File > Connect to Simulator**, then:

| What | Where in AdvantageScope |
|---|---|
| Robot on the field | 2D or 3D Field tab: `RealOutputs/Odometry/Robot` |
| Swerve wheels | Swerve tab: `RealOutputs/SwerveStates/Measured` and `SetpointsOptimized` |
| The arm | Mechanism tab: `RealOutputs/Arm/Mechanism2d` |

### Controls

The code uses position names (`faceDown`, `faceRight`, ...), so any gamepad works:

| Input | Xbox | DualSense | Does |
|---|---|---|---|
| Left stick | | | Drive (field-relative) |
| Right stick, left/right | | | Turn |
| Hold `faceDown` | A | Cross | Keep facing downfield (toward the other alliance) while driving |
| `faceLeft` | X | Square | Lock wheels in an X so the robot is hard to push. Moving the sticks unlocks them |
| Hold `faceRight` | B | Circle | Drive robot-relative: stick forward is the robot's own forward, not away from the driver |
| Hold left bumper | LB | L1 | Keep facing the alliance's hub (the 2026 game's goal) while driving |
| Hold right bumper | RB | R1 | Drive to the spot 1 m in front of the nearest AprilTag, facing it. The controller buzzes when it arrives |
| Hold left trigger | LT | L2 | Intake until a game piece is in. The robot starts holding one, so eject it first; after that the simulator always has another ready |
| Hold right trigger | RT | R2 | Eject the game piece |
| D-pad up / right / down | | | Arm to vertical / horizontal / stowed (folded down) |

### Things to try

- Run an autonomous routine. The **Auto Choices** and **Starting Position** choosers show up in any NetworkTables dashboard, such as Elastic. Pick one, then switch the Sim GUI to Autonomous and enable.
- Change the arm's gains while it runs, and watch what happens: [Tuning gains](docs/tuning.md).
- Graph the battery voltage while flooring it from a standstill: [The battery and brownouts](docs/battery-and-brownouts.md).
