# The drive and odometry

How a stick push becomes four wheel commands, and how the robot works out where it is from what the wheels and gyro report. It assumes the IO-layer idea from [How the code works](how-it-works.md) and the words in the [glossary](glossary.md). The vendor documentation linked at the end goes deeper on each piece.

WPILib 2027 renamed a few classes that most guides still use the old names for: `ChassisSpeeds` is now `ChassisVelocities`, and `SwerveModuleState` is now `SwerveModuleVelocity`. They work the same way.

## The pieces

All in `subsystems/drive/`:

| File | Job |
| --- | --- |
| `Drive` | The mechanism. Turns a robot velocity into module setpoints, and runs odometry and the pose estimator. |
| `Module` | One module. Converts between the IO layer's wheel radians and meters, and optimizes each setpoint. |
| `ModuleIO`, `ModuleIOTalonFX` | The IO layer for one module: two Talon FX motor controllers (drive and steer) and a CANcoder. |
| `GyroIO`, `GyroIOPigeon2` | The IO layer for the Pigeon 2 gyro. |
| `PhoenixOdometryThread` | Samples wheel and gyro positions 250 times a second, off the main loop. |
| `DriveConstants` | CAN IDs, gear ratios, gains, current limits, speeds, and the robot's size. |
| `DriveCommands` | Joystick driving, aiming, and driving to a pose. |

## Driving: from sticks to wheels

### Shaping the request

`DriveCommands.joystickDrive` turns the sticks into a velocity request:

1. **Deadband and squaring.** Stick movement smaller than the deadband counts as zero, and the rest is squared, so small pushes give fine control and a full push still gives full speed. The deadband applies to the stick's distance from center rather than to each axis, so diagonals don't snap to straight lines.
2. **Rate limiting.** `VectorRateLimiter` caps how fast the requested velocity can change, with a separate, higher limit for slowing down. It limits the velocity as one vector, so a diagonal start stays diagonal.
3. **Field-relative conversion.** The sticks ask for field directions ("away from the driver"), but the wheels need the robot's own directions. `ChassisVelocities.toRobotRelative(heading)` rotates the request by the robot's heading. This is the one line that makes driving field-relative, and it's why a wrong heading makes the robot drive the wrong way.

### Kinematics

`Drive.runVelocity` takes a robot-relative velocity (forward, left, and turning speed) and works out what each wheel must do:

1. **Discretize.** The command applies for a whole 20 ms loop, but the robot turns during that time, so a request to drive straight while spinning would come out as a curve. `ChassisVelocities.discretize` adjusts the request so the motion over the loop comes out as asked.
2. **Inverse kinematics.** `SwerveDriveKinematics.toSwerveModuleVelocities` gives each module a speed and an angle. Each module's velocity is the robot's velocity plus the extra speed from spinning, which is faster the farther the module is from the center. That's why the kinematics needs `MODULE_TRANSLATIONS`, each module's position on the robot.
3. **Desaturate.** If any wheel would need more than top speed, `desaturateWheelVelocities` slows every wheel by the same factor, so the robot keeps the requested direction and spin, only slower.![alt text](image.png)

### Per-module optimization

`Module.runSetpoint` makes two adjustments before sending a setpoint:

- **Optimize.** A wheel pointing at 0° asked for 170° can instead point at -10° and drive backward. `optimize` flips the setpoint whenever that's the shorter turn, so no module ever turns more than 90°.
- **Cosine scale.** While a module is still turning toward its angle, driving at full speed would push partly sideways. `cosineScale` multiplies the speed by the cosine of the remaining angle error: full speed when aligned, zero when 90° off.

### What the Talons do

Each module's two motors run their own control loops inside the Talon FX motor controllers, at about 1 kHz, much faster than the 50 Hz robot loop. The robot code only sends setpoints.

- **Drive motor:** a `VelocityVoltage` request. The Talon applies kS and kV feedforward plus kP feedback from `DRIVE_GAINS`. `SensorToMechanismRatio` is set to the drive gear ratio, so the Talon works in wheel rotations instead of motor rotations.
- **Steer motor:** a `PositionVoltage` request toward the module angle. `FusedCANcoder` makes the Talon use the CANcoder (which measures the module itself, after the gears) as its position sensor, and blends in the motor's own faster sensor between CANcoder readings. `ContinuousWrap` tells it that 359° and 1° are 2° apart, not 358°.
- **Current limits.** The stator limit (`SLIP_CURRENT_AMPS`) caps torque so the wheels don't spin out. The supply limit caps what each motor draws from the battery; see [The battery and brownouts](battery-and-brownouts.md).
- **Bevel coupling.** On the MK5n, the drive gearing passes through the steering axis, so steering the module turns the wheel a little even with the drive motor still. `ModuleIOTalonFX` corrects both ways: it subtracts the coupled motion from the wheel's measured position and speed, and adds it to the drive velocity setpoint.

The Talons can also run in torque-current mode (`TorqueCurrentFOC`), which sets motor current instead of voltage, so the response doesn't change as the battery sags. It needs a Phoenix Pro license; `ClosedLoopOutput` in `DriveConstants` switches between them, and the gains differ between the two.

## Odometry: working out where the robot is

### The idea

Each module reports a `SwerveModulePosition`: the total distance its wheel has rolled, and which way it points. Between two readings, each wheel's change in distance, in its direction, says how that corner of the robot moved. Forward kinematics combines the four into how the whole robot moved (a `Twist2d`), and adding up those small moves gives the pose.

Heading comes from the gyro rather than the wheels. Wheels scrub sideways during turns, so heading from wheels drifts quickly; a gyro drifts very little. If the gyro disconnects, `Drive.periodic` falls back to the heading from the wheels, adding to the last good heading so there's no jump.

### Why a separate thread

At 4.5 m/s the robot covers 9 cm per 20 ms loop, and modules can change angle partway through. Adding up one reading per loop treats each 9 cm as a straight line at one wheel angle, and readings fetched at slightly different moments don't quite agree. Both errors add up over a match.

So `PhoenixOdometryThread` samples every wheel and the gyro together at 250 Hz (100 Hz on a CAN bus that isn't CAN FD):

- **Status signals.** Every Phoenix device sends its readings over CAN as status frames at a set rate. `setUpdateFrequency` raises the rate for only the signals odometry needs (drive position, steer position, gyro yaw), and `optimizeBusUtilization` turns off frames nothing uses, to keep the bus from filling up.
- **Waiting for a matched set.** On CAN FD, `BaseStatusSignal.waitForAll` blocks until every signal has a new frame, so one sample holds readings from nearly the same instant. Without CAN FD, Phoenix doesn't allow waiting on several signals at once, so the thread sleeps and then refreshes them all.
- **Timestamps.** Each sample is stamped with the time the readings were measured: the current time minus the frames' average CAN latency, as Phoenix reports it.
- **Queues.** The thread adds each value to a queue. Each loop, `ModuleIOTalonFX.updateInputs` drains the queues into the inputs as arrays (`odometryTimestamps`, `odometryDrivePositionsRad`, `odometryTurnPositions`), and `Drive.periodic` runs through the samples oldest first.
- **Locks.** `Drive.odometryLock` keeps the thread from adding a sample while the main loop is draining the queues. Without it, one module could read 5 samples and the next 6.
- **Clones.** The thread works on its own copy (`clone()`) of each signal, so it never refreshes a signal at the same time the main loop does.

Because the samples arrive through the IO layer's inputs, AdvantageKit logs every one, and replay feeds them back unchanged. In replay the thread never starts: nothing registers a signal, so there's nothing to sample.

### The pose estimator

Instead of plain odometry, `Drive` uses `SwerveDrivePoseEstimator`, which does the same adding up and also takes in vision. Each odometry sample goes in with its own timestamp through `updateWithTime`. The estimator keeps a short history of recent poses, so when a camera result arrives, stamped with when the picture was taken, `addVisionMeasurement` corrects the pose from that moment and replays the odometry since then on top.

How far it moves toward each camera result depends on standard deviations: `ODOMETRY_STD_DEVS` for how much the wheels can be trusted, and one per camera result for vision (worked out in `Vision`). The smaller one wins more of the blend. MegaTag2 vision gets its heading from the gyro, so it's given an infinite heading standard deviation: it corrects position only, never heading.

`Drive.setPose` resets the estimator to a known pose, such as a starting position or a new heading from the driver. It doesn't move the gyro; the estimator just records the offset between the gyro's reading and the new heading.

## Where it goes wrong

Odometry counts what the wheels did, not what the robot did. Anything that moves the robot without rolling the wheels the same distance shows up as drift:

- **Wheel slip.** A wheel pushing harder than its grip spins faster than the ground moves. The stator current limit is set to stay under that.
- **Scrub.** In tight turns, wheels slide a little sideways, which they can't measure.
- **Wrong wheel radius.** Every distance comes out scaled by the error. `DriveCharacterization` has a routine that measures the real radius.
- **Wrong encoder offsets.** A module that thinks it points straight but doesn't drives slightly sideways. See `DriveCharacterization.findEncoderOffsets`.
- **Being pushed.** Another robot shoving this one sideways moves it with the wheels sliding, not rolling.

Vision is what pulls the pose back. In sim, `Sim/TruePose` shows where the simulated robot really is; graph it against `Odometry/Robot` to see how far odometry has drifted.

## References

CTRE Phoenix 6:

- [Status signals](https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/api-usage/status-signals.html): update frequencies, `refreshAll`, `waitForAll`, and latency.
- [Control requests](https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/api-usage/control-requests.html) and [closed-loop requests](https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/device-specific/talonfx/closed-loop-requests.html): `VelocityVoltage`, `PositionVoltage`, the torque-current versions, and how Slot0 gains are applied.
- [Remote sensors](https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/device-specific/talonfx/remote-sensors.html): `FusedCANcoder` and `RotorToSensorRatio`.
- [Current limits](https://v6.docs.ctr-electronics.com/en/stable/docs/hardware-reference/talonfx/improving-performance-with-current-limits.html): stator vs supply.

WPILib (these pages use the 2026 class names):

- [Swerve drive kinematics](https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html): inverse kinematics, desaturating, optimizing, and cosine scaling.
- [Swerve drive odometry](https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-odometry.html).
- [Pose estimators](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/state-space/state-space-pose-estimators.html): blending odometry and vision, and choosing standard deviations.

Others:

- [AdvantageKit TalonFX swerve template](https://docs.advantagekit.org/getting-started/template-projects/talonfx-swerve-template): the template this drive grew from, including its high-frequency odometry.
- [Limelight MegaTag2](https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization-megatag2): why MegaTag2 needs the robot's heading.

[Back to the README](../README.md)
