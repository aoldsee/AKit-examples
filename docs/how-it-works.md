# How the code works

The ideas the whole robot program is built on: IO layers, how the simulator plugs in, the robot loop, and field coordinates.

## The big idea: IO layers

AdvantageKit splits every mechanism into two halves:

- The **mechanism** (`Drive`, `Arm`) holds all the decisions: where to go, how fast, when it's done.
- The **IO** (`ModuleIO`, `GyroIO`, `ArmIO`) is the thin layer that actually talks to hardware. Its only jobs are "read the sensors into an inputs object" and "send this command to the motor."

Every loop, the mechanism asks its IO to fill in an inputs object, and AdvantageKit logs it. The mechanism only ever looks at those logged inputs, never at hardware directly. That rule is what makes **replay** possible: feed the logged inputs back in later, and the mechanism makes exactly the same decisions it made on the field. That means logging can be added or a bug fixed, and a real match rerun on a laptop.

Classes like `ArmIOInputsAutoLogged` aren't in `src/`: AdvantageKit generates them at build time from the inputs class marked `@AutoLog` (for example `ArmIO.ArmIOInputs`). They add the code that logs every field.

Which IO a mechanism gets depends on the mode the robot is running in. All of that choosing happens in one place, `RobotContainer`:

| Mode | When | Drive IO | Arm IO | Intake IO | Vision IO |
|---|---|---|---|---|---|
| REAL | On the robot | `ModuleIOTalonFX`, `GyroIOPigeon2` | `ArmIOTalonFX` | `IntakeIOTalonFX` | `VisionIOLimelight` |
| SIM | Simulator | `ModuleIOTalonFXSim`, `GyroIOPigeon2Sim` | `ArmIOTalonFXSim` | `IntakeIOTalonFXSim` | `VisionIOLimelight` |
| REPLAY | Replaying a log | Empty IO that does nothing | Empty IO | Empty IO | Empty IO |

## How simulation works here

CTRE's Phoenix library can pretend to be real motor controllers and sensors. So the sim IO classes are the real IO classes (they extend them) talking to pretend devices. `ArmIOTalonFXSim` is the whole idea in 13 lines. The configs, the motor controller's built-in control loops, and all our sensor handling run exactly as they would on the robot.

Something still has to play the part of the motors, gears, and gravity behind those pretend devices. That's the job of the **sim models**. In the diagram, arrows show which way information flows:

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
```

Every 4 ms, `SimWorld` moves the whole simulated world forward a step, on its own timer separate from the 20 ms robot loop (real physics doesn't wait for the robot code either): each model reads the voltage its simulated motor is applying, works out how the mechanism moves, and writes the new positions back into the simulated sensors. Each mechanism's model lives in its own folder, next to its IO.

The drive's model is worth a look as a pattern for any mechanism. It treats the robot as one body. Each step, every wheel works out how hard it pushes on the ground (from its motor's voltage, using the same kS, kV, and kA as the feedforward), the pushes add up to a total force and twist, and Newton's second law turns those into how the robot speeds up and turns: acceleration is force divided by mass, and spin-up is twist divided by the robot's moment of inertia. A wheel can't push harder than its grip allows (`WHEEL_COF` times the weight on it); past that, it slips, and odometry, which counts wheel turns, drifts from where the robot really is. `SwerveDriveSim` has the whole step in order.

Vision uses the same trick one level up. A real Limelight is a separate computer that publishes its results to NetworkTables, so `LimelightSim` pretends to be one: it looks at where the simulated robot really is, works out which AprilTags each camera could see, and publishes a result in Limelight's exact format. The real `VisionIOLimelight` reads it without knowing the difference.

## The robot loop

Every 20 ms, `Robot.robotPeriodic()` does two things, in this order:

1. `robotContainer.periodic()` updates every mechanism's inputs. Vision runs after the drive has updated its odometry, so camera corrections apply to this loop's pose rather than the last one's.
2. `scheduler.run()` runs the commands, which act on those fresh inputs.

Mechanisms don't run anything on their own each loop, so the robot calls each one's `periodic()` itself. Adding a mechanism means adding it to `RobotContainer.periodic()`.

## Field coordinates and alliances

Field positions always use the same frame, whichever alliance the robot is on:

```
 +y
  ^ BLUE wall                                        RED wall
  |    +-------------------------------------------------+
  |    |                                                 |
  |    |  blue side                           red side   |
  |    |                                                 |
  |    +-------------------------------------------------+
  |   (0,0) ----------------------------------------------> +x  (about 16.5 m long)
```

The origin is the blue alliance wall's right corner (as the blue drivers see it), +x points toward red, and +y is the blue drivers' left. A red robot sitting at its own wall has an x near 16.5 m.

So code that means "our side" (starting positions, our hub) writes the blue version once and calls `FieldGeometry.flipIfRed`. The 2026 field is rotationally symmetric, so flipping turns the field 180° about its center: x, y, and heading all change. Everything that depends on where the origin is or how the field flips lives in `field/FieldGeometry.java`.

Joystick driving flips too, but differently: on red the driver faces the other way down the field, so "stick forward" has to mean -x. That's `driverRelativeHeading` in `DriveCommands`.

[Back to the README](../README.md)
