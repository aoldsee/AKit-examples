# AKit-Base

A small but complete robot program for learning how a modern FRC robot is put together. It has a swerve drivetrain, a single-jointed arm, and AprilTag vision, logs everything with AdvantageKit, and runs in simulation on a laptop, where it can be driven, broken, and examined afterward without a real robot.

It targets WPILib 2027 (alpha 7) on SystemCore, CTRE Phoenix 6 motor controllers, and the new Commands v3 framework. Everything here is a placeholder robot: the hardware values are realistic but not measured from any real machine.

## Getting started

### Requirements

- The WPILib 2027 alpha 7 (or, probably, actual WPILib 2027)

### Run the simulator

1. In VS Code, run **WPILib: Simulate Robot Code**.
2. When it asks which extensions to use, tick **Sim GUI**. That window is the simulated Driver Station.
3. In the Sim GUI, drag the controller from **System Joysticks** onto **Joystick[0]**.
4. Set the robot state to **Teleoperated** and click **Enable**. The robot now drives.

To see the robot, open AdvantageScope and choose **File > Connect to Simulator**. Good things to look at first:

| What | Where in AdvantageScope |
|---|---|
| Robot on the field | 2D or 3D Field tab: `RealOutputs/Odometry/Robot` |
| Where the sim robot *really* is | Same tab, add `RealOutputs/Sim/TruePose` as a ghost |
| Swerve wheels | Swerve tab: `RealOutputs/SwerveStates/Measured` and `SetpointsOptimized` |
| The arm | Mechanism tab: `RealOutputs/Arm/Mechanism2d` |
| Game piece in the intake | `RealOutputs/Intake/HasPiece`, and the raw sensor at `Intake/SensorBlocked` |
| Battery voltage and current | `RealOutputs/Sim/Battery/Volts`, `TotalCurrentAmps`, and `CurrentAmps/Drive`, `Arm`, ... |
| What the cameras see | 3D Field tab: `RealOutputs/Vision/Summary/RobotPosesAccepted` (pose estimates) and `RealOutputs/Vision/Camera0/TagPoses` (tags in view) |
| Why vision estimates were thrown out | `RealOutputs/Vision/Summary/Rejected/...`, one running count per reason |
| Raw sensor values | `Drive/Module0` to `Module3`, `Drive/Gyro`, `Arm`, `Intake` |
| Motor faults since startup (brownouts, overheating, ...) | `Drive/Module0/DriveFaults` and friends, `Arm/MotorFaults`, `Intake/MotorFaults`. Any that are set also show up as alerts |

### Controls

The code uses position names (`faceDown`, `faceRight`, ...), so any gamepad works:

| Input | Xbox | DualSense | Does |
|---|---|---|---|
| Left stick | | | Drive (field-relative) |
| Right stick, left/right | | | Turn |
| Hold `faceDown` | A | Cross | Keep facing downfield while driving |
| `faceLeft` | X | Square | Lock wheels in an X so the robot is hard to push |
| `faceRight` | B | Circle | "The robot is facing away from me now" (resets heading) |
| Hold left bumper | LB | L1 | Keep facing the alliance's hub while driving |
| Hold right bumper | RB | R1 | Drive to the spot 1 m in front of the nearest AprilTag, facing it. The controller buzzes when it arrives |
| Hold left trigger | LT | L2 | Intake until a game piece is in |
| Hold right trigger | RT | R2 | Eject the game piece |
| D-pad up / right / down | | | Arm to vertical / horizontal / stowed |

## How the code is organized

### The big idea: IO layers

AdvantageKit splits every mechanism into two halves:

- The **mechanism** (`Drive`, `Arm`) holds all the decisions: where to go, how fast, when it's done.
- The **IO** (`ModuleIO`, `GyroIO`, `ArmIO`) is the thin layer that actually talks to hardware. Its only jobs are "read the sensors into an inputs object" and "send this command to the motor."

Every loop, the mechanism asks its IO to fill in an inputs object, and AdvantageKit logs it. The mechanism only ever looks at those logged inputs, never at hardware directly. That rule is what makes **replay** possible: feed the logged inputs back in later, and the mechanism makes exactly the same decisions it made on the field. That means logging can be added or a bug fixed, and a real match rerun on a laptop.

Which IO a mechanism gets depends on the mode the robot is running in. All of that choosing happens in one place, `RobotContainer`:

| Mode | When | Drive IO | Arm IO | Intake IO | Vision IO |
|---|---|---|---|---|---|
| REAL | On the robot | `ModuleIOTalonFX`, `GyroIOPigeon2` | `ArmIOTalonFX` | `IntakeIOTalonFX` | `VisionIOLimelight` |
| SIM | Simulator | `ModuleIOTalonFXSim`, `GyroIOPigeon2Sim` | `ArmIOTalonFXSim` | `IntakeIOTalonFXSim` | `VisionIOLimelight` |
| REPLAY | Replaying a log | Empty IO that does nothing | Empty IO | Empty IO | Empty IO |

### How simulation works here

This project simulates *below* the IO layer, which is a little different from most AdvantageKit examples.

CTRE's Phoenix library can pretend to be real motor controllers and sensors. So the sim IO classes are the real IO classes (they extend them) talking to pretend devices. The configs, the motor controller's built-in control loops, and all our sensor handling run exactly as they would on the robot.

Something still has to play the part of the motors, gears, and gravity behind those pretend devices. That's the job of the **sim models**:

```
 Robot code                Phoenix sim devices           Sim models (physics)
 ----------                -------------------           --------------------
 Drive  -> ModuleIOTalonFXSim -> simulated Talon FX  <->  SwerveModuleSim  \
                                 simulated CANcoder                         > SwerveDriveSim
        -> GyroIOPigeon2Sim  -> simulated Pigeon 2   <-------------------- /
 Arm    -> ArmIOTalonFXSim   -> simulated Talon FX   <->  ArmSim
                                 simulated CANcoder
 Intake -> IntakeIOTalonFXSim -> simulated Talon FX  <->  IntakeSim
                                 simulated DIO
 Vision -> VisionIOLimelight -> NetworkTables        <--  LimelightSim (one per camera)
                                                          SimWorld steps every model every 4 ms
```

Each model reads the voltage its simulated motor is applying, works out how the mechanism moves, and writes the new positions back into the simulated sensors. `SimWorld` runs every model together on one loop. Each mechanism's model lives in its own folder, next to its IO.

Vision uses the same trick one level up. A real Limelight is a separate computer that publishes its results to NetworkTables, so `LimelightSim` pretends to be one: it looks at where the simulated robot really is, works out which AprilTags each camera could see, and publishes a result in Limelight's exact format. The real `VisionIOLimelight` reads it without knowing the difference.

### The robot loop

Every 20 ms, `Robot.robotPeriodic()` does two things, in this order:

1. `robotContainer.periodic()` updates every mechanism's inputs. Vision runs right after the drive, so its corrections land on this loop's odometry.
2. `scheduler.run()` runs the commands, which act on those fresh inputs.

In Commands v3, mechanisms don't get an automatic `periodic()` like v2 subsystems did, so the robot has to call each one itself. Adding a mechanism means adding it to `RobotContainer.periodic()`.

### Commands v3 quick overview

A command is a small function that runs a little each loop. The important pieces:

- `mechanism.runRepeatedly(() -> ...)` runs the lambda every loop, forever. The joystick drive command works this way.
- `mechanism.run(coroutine -> { ... })` runs code that can **pause** between loops with `coroutine.yield()`. That lets a multi-step command read top to bottom, like a normal method:

  ```java
  // From Arm.goTo: set a goal, then keep commanding it each loop until we arrive.
  return run(coroutine -> {
        goalRad =
            Math.clamp(angleRad, ArmConstants.SOFT_MIN_ANGLE_RAD, ArmConstants.SOFT_MAX_ANGLE_RAD);
        while (!isAtGoal()) {
          io.setPosition(goalRad);
          coroutine.yield(); // pause until the next loop
        }
      })
      .named("Arm.GoTo[...]");
  ```

- `coroutine.await(otherCommand)` runs another command and waits for it to finish. `coroutine.awaitAll(a, b)` runs several at once. The example autos in `RobotContainer` show both.
- A **default command** runs whenever nothing else is using that mechanism. The drive's default is joystick driving; the arm's is "hold the current angle."
- A `Trigger` turns a condition (a button, or `arm.atGoal`) into something commands can be bound to. Triggers combine: the controller rumble is `rightBumper().and(robot is lined up)`, bound with `onTrue`, so it buzzes once when both become true.

Every command has a name (`.named(...)`), and those names appear in the logs.

`Intake.intake()` and `Intake.eject()` finish when a sensor changes instead of when a position is reached: "run the rollers until the beam break sees a piece." Most game-piece handling works this way.

`DriveCommands.driveToPose` is a good one to read next. It drives to a spot on the field and finishes once it's there and stopped, so it can be awaited like `Arm.goTo`. The "Align to Nearest Tag While Raising Arm" auto uses `awaitAll` to run both at once, which is how a real scoring routine gets built. Watch `DriveToPose/DistanceError` while it runs. "Score Preload at Nearest Tag" goes one step further: drive and raise the arm together, eject, then stow.

### Field coordinates and alliances

Field positions always use the same frame, whichever alliance the robot is on: the origin is the blue alliance wall's right corner (as the blue drivers see it), +x points toward red, and +y is the blue drivers' left. A red robot sitting at its own wall has an x near 16.5 m.

So code that means "our side" (starting positions, our hub) writes the blue version once and calls `FieldGeometry.flipIfRed`. The 2026 field is rotationally symmetric, so flipping turns the field 180° about its center: x, y, and heading all change. Some years mirror the field instead, and then only x and heading change. Everything that depends on where the origin is or how the field flips lives in `util/FieldGeometry.java`.

Joystick driving flips too, but differently: on red the driver faces the other way down the field, so "stick forward" has to mean -x. That's `driverRelativeHeading` in `DriveCommands`.

## Logs and replay

In simulation the robot writes a log to `logs/` every run (`.wpilog` files). On a real robot they go to a USB stick. Open any log in AdvantageScope with **File > Open Log**.

To replay a log:

1. In `Constants.java`, change `simMode` to `Mode.REPLAY`.
2. Open the log to replay in AdvantageScope.
3. Run **Simulate Robot Code**, but don't tick Sim GUI this time; replay needs every sim extension turned off. AdvantageKit picks up the log that's open in AdvantageScope. Setting the `AKIT_LOG_PATH` environment variable to a log's path works too.
4. Replay writes a new log ending in `_sim.wpilog`. Its `ReplayOutputs` are what the current code decides when fed the old inputs, side by side with the original `RealOutputs`.
5. Set `simMode` back to `Mode.SIM` afterward.

## Tests

There are two kinds of tests, but they have different purposes.

**Unit tests** check logic by itself. They hand a mechanism a fake IO (a few lines of code that pretends to be the hardware), set up a situation, and check what the mechanism decides. For example, `ArmTest` tells the arm it's resting on the hard stop, enables the robot, and checks that the arm doesn't try to lift itself. No motors, physics, or waiting, so they finish in a fraction of a second. This is the IO layer paying off again: the same split that makes replay possible makes the logic easy to test.

**Simulation tests** (tagged `sim`) run the real robot code against the simulator, in real time, because the simulated Phoenix devices run on a real clock. They check that everything works together: the drive goes where it's told, odometry agrees with where the robot really is, characterization measures the values the sim was built with, the arm reaches its targets and holds against gravity, and vision pulls a wrong pose estimate back to the truth. The suite takes about a minute.

```bash
./gradlew unitTest   # just the fast unit tests, a few seconds
./gradlew test       # everything, about a minute
```

`./gradlew build` runs neither, to keep builds quick. Run the tests before pushing. GitHub Actions runs all of them on every push either way.

## The battery and brownouts

A brownout is when the battery voltage drops so low the robot controller can't keep everything running. On SystemCore that line is 6.75 V, and the robot stays browned out until the voltage climbs back above 7.25 V.

The cause is almost always current. A battery behaves like a voltage source with a small resistor inside it (about 0.02 ohms, counting wires and connectors). Every amp the robot draws drops the voltage by amps times that resistance, so 300 A costs about 6 V. That resistor also means a battery has a maximum power it can ever deliver, about 1800-2000 W for FRC batteries. Four drive motors running flat out can ask for nearly twice that.

The simulator models this. Each mechanism's sim model reports how much current it's drawing, `SimWorld` adds them up and works out the sagging voltage, and every simulated motor then runs on that lower voltage. Graph `Sim/Battery/Volts` next to `Sim/Battery/CurrentAmps/Drive` in AdvantageScope and floor it from a standstill to see it happen. If the voltage dips below 6.75 V, the "Simulated brownout" alert fires. The robot's weight matters here, so set `robotMassKg` in `Constants.java` to the real robot's.

### Supply current limits

The main fix is a **supply current limit**: a cap on how much current each motor may pull from the battery. (A stator current limit caps the current in the motor itself, which is about torque and heat; the supply limit is about the battery.) `TunerConstants.kDriveSupplyCurrentLimit` sets it for the drive motors.

A supply limit is a smart tool because a motor draws little from the battery at low speed even while making full torque. Battery current tracks power, and power is torque times speed. The limit only bites at speed, where the battery is actually hurting, so the robot still launches and pushes hard.

### Acceleration (slew) limits

The joystick drive commands also limit how quickly the robot's velocity can change (`util/VectorRateLimiter.java`). This is a blunter tool for brownouts, because it limits torque at every speed, including low speeds where torque costs the battery almost nothing. Where it earns its place is everything a supply limit doesn't catch: the instant current spike when the stick is slammed (the supply limiter takes a moment to react), reversals from full forward to full back, wheel slip, tipping, and gearbox shock. Driving also just feels smoother.

Note:
- The limit applies to the whole velocity vector, not x and y separately. Limiting each axis on its own makes diagonal moves curve.
- Slowing down has its own, higher limit, so the driver can always stop quickly. Reversing counts as slowing down until the robot passes through zero.

A blocked robot still pushes at full strength: its velocity setpoint ramps up to full anyway, and the motors stay at their current limit.

## Tuning gains live

With `tuningMode` on in `Constants.java` (it is by default), the arm's gains show up in NetworkTables under `/Tuning/Arm/`: `kP`, `kD`, `kG`, `CruiseVelocity`, and `Acceleration`. The drive's acceleration limits are under `/Tuning/Drive/`: `MaxAccelMetersPerSec2` and `MaxDecelMetersPerSec2`. Change them from Elastic or AdvantageScope while the robot (or simulator) is running, and the arm picks up the new values right away. Things to try in the simulator:

- Set `kG` to 0 and watch the arm sag below its goal, especially near horizontal where gravity pulls hardest.
- Raise `kP` until the arm overshoots and rings, then add a little `kD` to calm it.
- Lower `Acceleration` and watch the moves get gentler.
- Set the drive's `MaxAccelMetersPerSec2` very high, floor it, and watch `Sim/Battery/CurrentAmps/Drive` spike; then bring it back down.

Because the values come in through AdvantageKit, they're recorded in the log as inputs, so replaying a run reproduces the gain changes too. Once the values look right, copy them into `ArmConstants.java`; the dashboard values reset when the code restarts. Turn `tuningMode` off for competition.

## Pit check

**Drive Systems Check** in the Auto Choices list checks that every swerve module works. Put the robot on blocks (or give it a couple of meters of clear floor), select it, and enable in autonomous. It steers every module to 90° and back, then spins every wheel forward on the same voltage, and takes about 3 seconds. The result shows up as an alert: either "passed", or a list of what failed and what to check (a module that didn't steer, a wheel spinning backward, a dead motor, a wheel much slower than the rest). It can't tell if a CANcoder offset is wrong, so at the end, look: every wheel should point straight ahead. `DriveSystemsCheckTest` shows it catching each problem with deliberately broken fake modules.

## Characterization (finding the gains)

The gains in this project are calculated from motor specs, which gets close but not exact. Characterization measures the real robot. Each routine is in the **Auto Choices** list. Run it in autonomous on a robot that has room to move.

| Routine | Measures | How to read the result |
|---|---|---|
| Drive FF Characterization | Drive kS and kV | Stops on disable; prints kS and kV in the console |
| Drive Wheel Radius Characterization | Real wheel radius (tread wears!) | Spins slowly in place; disable after at least one full turn, result prints |
| Drive SysId (4 routines) | kS, kV, kA | Run all four, then load the log into the SysId tool and pick `Drive/SysIdState` as the test state |

Put the results into `TunerConstants.java` (drive) or `ArmConstants.java` (arm).

## Moving to a real robot

This code has never driven real hardware, so expect to work through these:

1. **Robot mass and size.** Weigh the robot with bumpers and battery and set `robotMassKg` in `Constants.java`. Measure front to back over the bumpers and set `BUMPER_LENGTH_METERS` in `DriveConstants.java`; the starting positions use it.
2. **Hardware specs.** `util/MK5n.java` assumes R2 gearing and `util/Motors.java` the Kraken X60 and X44. If the hardware differs, change them there; gear ratios, top speed, and feedforward gains are all derived from these.
3. **Team number.** Set the team's number in `.wpilib/wpilib_preferences.json` (it's 9999 here, for development).
4. **Swerve constants.** Generate `TunerConstants.java` with Phoenix Tuner X's swerve generator if it supports 2027 by then, or edit the values by hand: CAN IDs, inverts, gear ratios, encoder offsets, module positions.
5. **Arm constants.** Set the CAN IDs, inverts, and gear ratio in `ArmConstants.java`. The CANcoder offset must make the arm read **0 when it's horizontal**, because the gravity feedforward assumes that.
6. **Intake constants.** Set the CAN ID, gear ratio, and beam break port in `IntakeConstants.java`. Then put a piece in by hand and watch `Intake/SensorBlocked`: if it reads false with the piece in, flip `SENSOR_INVERTED`.
7. **CAN buses.** The drivetrain is on SystemCore port `CAN_S0`, and the arm and intake on `CAN_S1`. Change them to match the wiring.
7. **Limelights.** Name them `limelight-front` and `limelight-back` in their web UI (or change `CAMERA_NAMES` in `VisionConstants.java`), measure where each camera sits on the robot and set `ROBOT_TO_CAMERA` in `VisionConstants.java`, and use a MegaTag2 AprilTag pipeline. The robot sends each camera its position at startup, overriding the web UI's camera pose. After deploying, check the web UI shows the same position; if the pitch is upside down, the sign conversion in `VisionIOLimelight.toLimelightCameraPose` is the place to look. Check the field layout in `FieldGeometry.FIELD` matches the current season.
9. **Field geometry.** The starting positions assume the three driver stations split the alliance wall evenly, with station 1 at the drivers' left. Check both against the game manual.
10. **Phoenix Pro.** This code uses Pro features (FOC commutation and fused CANcoders).
11. **Run the Drive Systems Check** on blocks until it passes and every wheel points straight ahead.
12. **Characterize**, then tune.
