# Terms and Java concepts / keywords

## Basics

- **Pose**: where the robot is and which way it faces (x, y, heading).
- **Field-relative** driving: stick forward always means "away from the driver", whichever way the robot faces. **Robot-relative** means "the robot's own forward".
- **Mechanism**: one part of the robot that commands control (the drive, the arm, the intake). It's a Java interface the mechanism classes implement. WPILib used to call these *subsystems*, which is why the folder is still `subsystems/`.
- **Setpoint** (or goal): where a controller is trying to get a mechanism, like an arm angle or a wheel speed.
- **PID**: the most common controller. Its **P** part (gain kP) pushes harder the farther the mechanism is from the setpoint; its **D** part (kD) pushes back against moving faster than planned (for a fixed setpoint, against moving at all), which damps wobble. (The **I** part is rarely needed and isn't used here.)
- **Coroutine**: a method that can pause partway through and pick up where it left off on the next loop. Commands are written as coroutines; see [Commands v3](commands.md).
- `() -> x` and `coroutine -> { ... }` are **lambdas**: small unnamed methods handed to another method, which calls them later. In `coroutine -> { ... }`, `coroutine` is the parameter. A lambda with just one statement can skip the braces: `coroutine -> drive.stop()` means the same as `coroutine -> { drive.stop(); }`.
- `drive::getRotation` is a **method reference**: it hands over the method itself (not its result), so it can be called later.

## Robot and control words

- **Swerve**: a drivetrain where each wheel can both drive and turn to point any direction, so the robot can move sideways and spin while driving.
- **AprilTag**: a printed black-and-white square on the field. A camera that sees one knows exactly which tag it is and where it is, so it can work out where the robot is.
- **Odometry**: working out the pose by adding up small wheel movements, many times a second.
- **Kinematics**: the math that turns "move the robot this way" into each wheel's speed and angle, and back.
- **Pose estimator**: the part of the drive that blends odometry with camera readings into one best guess of the pose.
- **Standard deviation** (std dev): roughly how far off a measurement might be. A bigger one means trust it less.
- **NetworkTables**: the shared table of values the robot, dashboards, and cameras use to talk to each other over the network.
- **Feedforward**: the push a controller can predict ahead of time, like the voltage to hold an arm up against gravity, instead of waiting for an error to appear. Its gains are kS, kV, kA, and kG; see [Characterization](characterization.md).
- **Motion profile**: a plan for a move: speed up, cruise, slow down, within set limits on speed and acceleration. A controller following a profile moves smoothly instead of lurching at full power. A **trapezoid profile** is named for the shape of its speed-over-time graph. **Motion Magic** is CTRE's name for a profile the motor controller runs by itself.
- **Open loop**: sending a motor a fixed voltage without checking where it got to. **Closed loop** means a controller keeps adjusting based on a sensor.
- **Radians**: the angle unit most of the math uses. A full turn is 2π, so 90° is about 1.57. Names ending in `Rad` are in radians.
- **Omega** (ω): turning speed, in radians per second.
- **Yaw**: which way the robot faces, as an angle; turning changes the yaw. The gyro measures it.
- **Frame**: "Frame of Reference"; which directions count as "forward" and "left". In the **field frame** they're fixed to the field; in the **robot frame** they turn with the robot. The **driver frame** is the field frame as the driver sees it, which is flipped on red.
- **Deadband**: a small zone around the joystick's center that counts as zero, so a stick that doesn't rest perfectly centered doesn't creep the robot.
- **Rate limiter**: caps how fast a value can change, like how quickly the robot can speed up.
- **Hard stop** and **soft limit**: a hard stop is the physical metal a mechanism rests against; a soft limit is a point a little before it, where the code stops so the motor never drives into the metal.
- **End effector**: whatever is on the end of an arm that handles the game piece, like this robot's intake rollers.
- **Beam break**: a sensor with a light beam across a gap. Anything in the gap blocks the beam, so it says whether a game piece is there.
- **Stator** and **supply** current: stator current flows inside the motor and decides how hard it pushes; supply current is what it takes from the battery. They can be very different; see [The battery and brownouts](battery-and-brownouts.md).
- **MOI** (moment of inertia): how hard something is to spin up, the turning version of mass.
- **Back-EMF**: a spinning motor also works as a generator, making a voltage that pushes against the voltage driving it. The faster it spins, the more it pushes back, which is why a motor has a top speed and why kV exists.
- **Plant**: control-theory word for the thing being controlled, like the arm or a wheel. A plant model is the math for how it responds to voltage.
- **Discretize**: turn a smooth, continuous request into the right one for a fixed time step. The drive does it because the robot turns during each 20 ms loop.
- **MegaTag1** and **MegaTag2**: the Limelight's two ways of finding the robot from AprilTags. MegaTag1 works out position and heading from the image alone. MegaTag2 uses the robot's own gyro heading instead, which makes its position much steadier, but it can't correct that heading. The code uses MegaTag2 for position and MegaTag1 for heading.
- **Alert**: a warning shown on the dashboard (and logged), like "Disconnected arm motor".

## Hardware and software names

- **SystemCore**: the robot controller, the computer that runs this code. It replaces the roboRIO.
- **CAN bus**: the pair of wires that connects the robot controller to motor controllers and sensors. Each device sends its readings as small messages called **status frames**, a set number of times a second.
- **Talon FX**: the motor controller built into the Kraken motors. **CANcoder**: an angle sensor (each swerve module has one to know which way its wheel points). **Pigeon 2**: the gyro, which measures which way the robot faces. All three are made by CTRE and talk over the CAN bus.
- **FOC** (field-oriented control): a smarter way for the Talon to drive its motor that gets more torque out of it. It needs a Phoenix Pro license.
- **CAN FD**: a faster version of the CAN bus that carries more data per message, so devices can send readings more often.
- **Tuner X**: CTRE's app for setting up, testing, and updating CTRE devices.
- **Hoot log**: CTRE's log file format. Phoenix can record every device signal into one, alongside the AdvantageKit log.
- **Vendordep** (vendor dependency): a file in `vendordeps/` that adds a company's library (like Phoenix 6) to the project.
- **DIO**: a digital input/output port, for simple on/off sensors like the intake's beam break.
- **Limelight**: a smart camera that finds AprilTags and works out where the robot is.
- **AdvantageKit**: the library that logs every input and output, which makes replay possible. **Phoenix 6**: CTRE's library for its devices. **Commands v3**: WPILib's newest way of writing commands; see [Commands v3](commands.md).

## Java

- The `coroutine` parameter in a command is an object the command system hands in. Its methods (`yield`, `wait`, `await`) are how a command pauses. Commands that never pause still get one, and just don't use it.
- `List<LimelightSim>`, `Supplier<Pose2d>`, and other `<...>` types are **generics**: the part in angle brackets says what kind of thing is inside. A `List<LimelightSim>` is a list of LimelightSims.
- `DoubleSupplier` and `Supplier<Pose2d>` mean "something to call for a fresh value". They're used instead of a plain number when the value has to be read again every loop, like a joystick axis.
- `Consumer<Pose2d>` is the other direction: something to hand a value to. `resetPose.accept(pose)` calls it with `pose`.
- `var armSim = new ArmSim();` lets Java work out the type from the right-hand side.
- `record Gains(double kP, double kD, ...)` is a small class that only holds values. Read them with `gains.kP()`. Two records are `.equals` when all their values match.
- `switch` with `case REAL -> ...` is the newer switch: no `break` needed, and it can produce a value. The older form, `case REAL: ... break;`, also appears; there, two cases written one after the other (`case REAL: case SIM:`) share the same code.
- `default` methods give an interface method a built-in body. The IO interfaces use empty ones, so `new ArmIO() {}` (an *anonymous class*) is an IO that does nothing.
- A class written inside an interface, like `IntakeIO.IntakeIOInputs`, is just a class that lives there because it belongs with that interface.
- No `public`, `private`, or `protected` in front of something makes it **package-private**: only code in the same folder (package) can use it.
- `import static first.robot.util.PhoenixUtil.tryUntilOk;` lets a file call `tryUntilOk(...)` without the class name in front.
- A **thread** is a separate line of execution that runs at the same time as the main robot loop. The drive's odometry thread is one. When two threads use the same data, a **lock** makes one wait while the other is using it, so neither sees the data half-changed.
- `try { ... } finally { ... }`: the `finally` part always runs, even if something goes wrong in the `try` part. Used to make sure a lock is always released.
- `Optional<Pose2d>` is "maybe a pose". `.orElse(something)` gives a fallback when there isn't one, and `.orElseThrow()` stops the program with an error instead.
- `@Override`, `@AutoLog`, and other `@` words are **annotations**: labels that tools read. `@AutoLog` tells AdvantageKit to generate the logging code for an inputs class, and `@AutoLogOutput` on a method logs what it returns, every loop.
- `Units.Seconds.of(1.0)` is a number with its unit attached, so a method that wants a time can't be handed a distance by mistake. Careful: there are two classes named `Units`. `org.wpilib.units.Units` makes these numbers-with-units; `org.wpilib.math.util.Units` has plain conversions like `Units.degreesToRadians(90)`. The `import` at the top of the file says which one it is.
- `list.forEach(x -> ...)` runs the lambda once for each item in the list.
- `VisionIO... io` (three dots) lets a method take any number of arguments of that type; inside, `io` is an array.
- `Double.NaN` means "not a number". The code uses it as "no value yet".
- `x ? a : b` is the **ternary** operator: `a` if `x` is true, otherwise `b`.
- `1e-6` is 0.000001 (1 times 10 to the -6). The code uses tiny numbers like this as "basically zero".

[Back to the README](../README.md)
