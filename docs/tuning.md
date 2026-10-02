# Tuning gains

Gains are the numbers that decide how hard a controller pushes. Every closed-loop mechanism on this robot uses the same few kinds, so tuning one teaches all of them.

## The gains

**Feedback** reacts to error: the difference between where a mechanism is and where it should be.

- **kP**: how hard to push per unit of error. Too low and the mechanism is sluggish or stops short; too high and it overshoots and wobbles.
- **kD**: pushes back against moving faster than planned (for a fixed goal, against moving at all), which damps the wobble kP causes.
- **kI**: pushes harder the longer an error lasts, by adding the error up over time. It removes a small error that never goes away on its own, like a mechanism stopping just short of its goal because of a load no other gain accounts for. It's rarely the right tool for robot movement, and every gain here leaves it at 0. Most errors that last come from something predictable, like gravity or friction, and feedforward (kG, kS) cancels those right away. kI only builds up after the error has already been sitting there. It also misbehaves when the mechanism can't move: against a hard stop, or held by another robot, the sum keeps growing ("windup"), and when the mechanism is freed it overshoots hard. Where kI is used, it's usually limited to small errors near the goal (an "I-zone") or given a cap on how big the sum can get.

**Feedforward** predicts the push before any error appears: kS for friction, kV for speed, kA for acceleration, and kG for gravity. These come from measuring the mechanism; see [Characterization](characterization.md).

**Motion profile limits** (a top speed and an acceleration) shape each move instead of pushing harder or softer. Lower limits mean gentler moves, less current, and less wobble, at the cost of time.

### Units

A gain's units come from what its error is measured in, so the same number means very different things in different places. The arm's kP is volts per rotation of the arm. The drive wheels' kP is volts per wheel rotation/s of speed error. The heading controller's kP in `DriveCommands.java` is rad/s of turning per radian of heading error, because its output is a speed request rather than volts. Always check the units before comparing a gain with someone else's.

## Order

The same order works for any mechanism:

1. Feedforward first. With good feedforward, the mechanism nearly follows its plan with no feedback at all, and feedback only cleans up what's left.
2. Raise kP until the mechanism follows closely and starts to overshoot, then back off a little.
3. Add kD if it still rings.
4. Adjust the profile limits for how fast the moves should be.
5. Only if a small, lasting error remains and no feedforward explains it, try a little kI.

## Where the gains are

- Arm: `GAINS` in `ArmConstants.java`.
- Drive wheels and steering: the `Slot0Configs` in `DriveConstants.java`. These run inside the Talons, on their own fast loop.
- Heading and alignment controllers: the `*_KP` and `*_KD` constants at the top of `DriveCommands.java`.

## Changing gains live

With `TUNING_MODE` on in `Constants.java` (it is by default), some values show up in NetworkTables, and the code picks up changes from a dashboard such as Elastic right away, while the robot or simulator runs:

- `/Tuning/Arm/`: the arm's `kP`, `kD`, `kG`, `CruiseVelocity`, and `Acceleration`.
- `/Tuning/Drive/`: the joystick drive's `MaxAccelMetersPerSec2` and `MaxDecelMetersPerSec2`.

Other gains are fixed until the code restarts. Making one live takes a `TunableNumber`, the same way `Arm.java` does it.

Because the values come in through AdvantageKit, they're recorded in the log as inputs, so replaying a run reproduces the gain changes too. The dashboard values reset when the code restarts, so copy good values back into the constants. Turn `TUNING_MODE` off for competition.

## Things to try in the simulator

- Set the arm's `kP` to 5, then `kG` to 0, and watch the arm sag below its goal (about 15° at horizontal, where gravity pulls hardest). At the normal `kP` of 50 the sag is only about 1.5°: a stiff `kP` hides a missing `kG`. kI would also pull the arm back up eventually, but slowly, and the fix that matches the cause is `kG`. Graph `RealOutputs/Arm/GoalRad` against `Arm/PositionRad` to see it exactly.
- Raise the arm's `kP` until it overshoots and rings, then add a little `kD` to calm it.
- Lower the arm's `Acceleration` and watch the moves get gentler.
- Set the drive's `MaxAccelMetersPerSec2` very high, floor it, and watch `RealOutputs/Sim/Battery/CurrentAmps/Drive` spike; then bring it back down.

[Back to the README](../README.md)
