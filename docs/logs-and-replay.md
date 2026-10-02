# Logs and replay

## What to look at

Connect AdvantageScope to the simulator (**File > Connect to Simulator**) or open a log (**File > Open Log**). Values under `RealOutputs/` are what the code worked out; the others are raw sensor readings. ("Real" is AdvantageKit's name for "this run", as opposed to a replay, so it's used in the simulator too.)

| What | Where in AdvantageScope |
|---|---|
| Robot on the field | 2D or 3D Field tab: `RealOutputs/Odometry/Robot` |
| Swerve wheels | Swerve tab: `RealOutputs/SwerveStates/Measured` and `SetpointsOptimized` |
| The arm | Mechanism tab: `RealOutputs/Arm/Mechanism2d` |
| Where the sim robot *really* is | Field tab, drag `RealOutputs/Sim/TruePose` onto the field and pick "Ghost" |
| Game piece in the intake | `RealOutputs/Intake/HasPiece`, and the raw sensor at `Intake/SensorBlocked` |
| Battery voltage and current | `RealOutputs/Sim/Battery/Volts`, `TotalCurrentAmps`, and `CurrentAmps/Drive`, `Arm`, ... |
| What the cameras see | 3D Field tab: `RealOutputs/Vision/Summary/RobotPosesAccepted` (pose estimates) and `RealOutputs/Vision/Camera0/TagPoses` (tags in view) |
| Why vision estimates were thrown out | `RealOutputs/Vision/Summary/Rejected/...`, one running count per reason |
| Raw sensor values | `Drive/Module0` to `Module3`, `Drive/Gyro`, `Arm`, `Intake` |
| Motor faults since startup (brownouts, overheating, ...) | `Drive/Module0/DriveFaults` and friends, `Arm/MotorFaults`, `Intake/MotorFaults`. Any that are set also show up as alerts |

## Logs

In simulation the robot writes a log to `logs/` every run (`.wpilog` files). On a real robot they go to a USB stick. Open any log in AdvantageScope with **File > Open Log**.

To replay a log:

1. In `Constants.java`, change `SIM_MODE` to `Mode.REPLAY`.
2. Open the log to replay in AdvantageScope.
3. Run **Simulate Robot Code**, but don't tick Sim GUI this time; replay needs every simulator extension turned off. AdvantageKit picks up the log that's open in AdvantageScope.
4. Replay writes a new log ending in `_sim.wpilog`. Its `ReplayOutputs` are what the current code decides when fed the old inputs, side by side with the original `RealOutputs`.
5. Set `SIM_MODE` back to `Mode.SIM` afterward.

[Back to the README](../README.md)
