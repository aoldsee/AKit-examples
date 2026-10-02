# Characterization (finding the gains)

Real-robot work: skip this until there's a robot to measure.

The gains in this project are calculated from motor specs, which gets close but not exact. Characterization measures the real robot instead. It finds the **feedforward** gains, which predict the voltage a move needs before any error shows up:

- **kS**: volts just to overcome friction
- **kV**: volts per unit of speed
- **kA**: volts per unit of acceleration
- **kG** (arm only): volts to hold up against gravity

Each routine is in the **Auto Choices** list. Run it in autonomous on a robot with a long stretch of open floor, and disable before it runs out of room: these routines keep speeding up until stopped. The feedforward ramp covers about 7 m in its first 20 seconds and keeps accelerating; the SysId tests reach 3 to 4 m/s. SysId only needs a few seconds of data from each test, so it's fine to stop them early.

| Routine | Measures | How to read the result |
|---|---|---|
| Drive FF Characterization | Drive kS and kV | Stops on disable; prints kS and kV in the console |
| Drive Wheel Radius Characterization | Real wheel radius (tread wears!) | Spins slowly in place; disable after at least one full turn, result prints |
| Drive SysId (4 routines) | kS, kV, kA | Run all four, then load the log into the SysId tool (see below) |

Put the results into the characterization constants: `DRIVE_KS`, `DRIVE_KV`, and `DRIVE_KA` in `DriveConstants.java`, or `KS`, `KV`, `KA`, and `KG` in `ArmConstants.java`. The wheel radius goes in `WHEEL_RADIUS_METERS` (set in `hardware/MK5n.java`). Those numbers do two jobs:

- They're the feedforward gains, so the real robot's control gets them right. (The drive's kA is the exception: the drive commands don't send accelerations, so only the sim uses it.)
- They're also what the simulator builds its models from, so the sim behaves like the real robot. Gains tuned on the real robot (kP, kD) then work in sim too.

### Reading SysId results

In the SysId tool, pick `RealOutputs/Drive/SysIdState` as the test state, and one module's `Drive/Module0/DriveAppliedVolts`, `DrivePositionRad`, and `DriveVelocityRadPerSec` as the data. Those are in radians of wheel rotation, so SysId reports kV in volts per (radian/s) and kA in volts per (radian/s²). The constants here are per wheel *rotation*, so multiply kV and kA by 2π before putting them in. kS needs no change.

Treat the first moment of the dynamic tests with suspicion. A 7 V step from a standstill asks for far more current than the current limits allow, so the start of the run shows the limits, not the motor's natural response. On a real robot with worn tread, that much push can also slip the wheels. If kA looks off, lower `SYSID_STEP_VOLTAGE` in `DriveCharacterization.java`.

### Characterizing another mechanism

There's no ready-made routine for the arm. Writing one is a good project, and every characterization test has the same shape:

1. Drive the mechanism with a chosen voltage, open loop (no PID), so the measured response is the mechanism's own. `ArmIO.setVoltage` does this for the arm.
2. Log the voltage actually applied, the position, and the velocity. The arm's IO inputs already include all three.
3. Log which test is running. SysId needs a test-state entry like `Drive/SysIdState`, with values from `SysIdRoutineLog.State`.
4. Run four tests: a slow voltage ramp each way (quasistatic: speed follows voltage, which gives kS and kV) and a sudden voltage step each way (dynamic: the speed-up gives kA).
5. Stop before anything breaks. An arm has hard stops, so end each test when it nears a soft limit.

`DriveCharacterization.sysIdTest` is a template: copy it, call `arm.setVoltage` instead of the drive, check the angle each loop, and add the four tests to the auto chooser. In the SysId tool, choose the **Arm** mechanism type; it fits kG along with the rest, and needs the angle to read 0 at horizontal.

For a quick estimate of the arm's kS and kG without SysId: hold the arm horizontal, then slowly raise the voltage until it just starts moving up (call that V_up), and slowly lower it until it just starts moving down (V_down). Gravity is the middle of the two and friction is half the gap: kG = (V_up + V_down) / 2 and kS = (V_up − V_down) / 2.

The sim still can't model everything exactly:

- Where real life is known to differ, the sim has a `FUDGE_` constant, set to "no effect" by default: gyro drift in `SwerveDriveSim`, camera noise and bad frames in `LimelightSim`, and extra gravity on the arm in `ArmSim`. Adjust those to make the sim act like a particular real robot, rather than changing the robot's constants. Each one's comment suggests a value to try, to see what that effect does. Wheel slip isn't a fudge: the drive sim models grip directly, from `WHEEL_COF` in `DriveConstants`.
- Gear backlash isn't modeled; it would need a change to the sim's physics, not just a constant. A gain tuned hard against backlash on the real robot may act a little differently in sim.

[Back to the README](../README.md)
