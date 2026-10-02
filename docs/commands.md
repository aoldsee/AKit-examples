# Commands v3

How commands work in this project: what a command is, how it pauses between loops, how commands share mechanisms, and where they live in the code.

A command is a small piece of code that runs a little each loop. The important pieces:

- `mechanism.runRepeatedly(() -> ...)` runs the lambda every loop, forever.
- `mechanism.run(coroutine -> { ... })` runs code that can **pause** between loops with `coroutine.yield()`. That lets a multi-step command read top to bottom, like a normal method:

  ```java
  // From Intake.intake: run the rollers each loop until a piece is in.
  return run(coroutine -> {
        while (!hasPiece) {
          io.setVoltage(IntakeConstants.INTAKE_VOLTS);
          coroutine.yield(); // pause until the next loop
        }
      })
      .named("Intake.Intake");
  ```

  A loop like this doesn't freeze the robot: `yield()` hands control back until the next loop. Even `while (true)` is fine (the joystick drive command does it); that command just runs until it's canceled.
- **Requirements**: a command made with `mechanism.run(...)` *requires* that mechanism. Only one command can use a mechanism at a time, so starting a new one cancels whatever was using it. `Command.noRequirements(...)` makes a command that only coordinates other commands, like an auto routine. If an auto required the drive itself, starting the drive command inside it would cancel the auto.
- Code inside the `coroutine -> { ... }` lambda runs each time the command starts. Code in the method around it, before the `return`, runs once, when the command is built (usually at startup, in `Controls`). That's why `DriveCommands` creates its controllers outside the lambda and resets them inside: one controller, a fresh start each time the button is pressed.
- A `run(coroutine -> ...)` with no loop does its work once and finishes. The X-lock and heading-reset buttons work that way.
- Three ways to pause inside a command: `coroutine.yield()` waits one loop, `coroutine.wait(time)` waits that long, and `coroutine.await(otherCommand)` runs another command and waits for it to finish.
- `coroutine.awaitAll(a, b)` runs several commands at once, as long as they need different mechanisms. The routines in `autos/Autos.java` show both.
- A **default command** runs whenever no other command requires that mechanism. The drive's default is joystick driving; the arm's is "hold the last goal angle."
- A `Trigger` turns a condition (a button, or any true/false check) into something commands can be bound to. Every gamepad button is a Trigger, and `RobotContainer` makes one from a dashboard value for the calibration button. `onTrue` starts a command once when the condition becomes true and lets it finish. `whileTrue` runs it while the condition holds and cancels it when it stops. Triggers combine with `.and(...)` and `.or(...)`.
- A command can chain others. Holding RB runs a small routine in `Controls`: `await` the alignment, then `await` a rumble, so the controller buzzes exactly when the robot arrives.

Every command has a name (`.named(...)`), and those names appear in the logs.

Where commands live:

- A command that uses one mechanism belongs to it: `Arm.goTo` and `Intake.eject` are methods on the mechanism, and the drive's commands are in its package (`DriveCommands` for driving, `DriveCharacterization` and `DriveSystemsCheck` for measuring it).
- A routine that combines mechanisms goes in `autos/Autos.java`.
- `Controls` binds the gamepad's buttons to commands, and `RobotContainer` connects everything else: it picks IO and fills the auto chooser.

In `DriveCommands`, `driveToPose` is a good read after `joystickDrive`. It drives to a spot on the field and finishes once it's there and stopped, so it can be awaited like `Arm.goTo`. Read its top comment and the shape of its loop; the math in the middle is optional. The "Align to Nearest Tag While Raising Arm" auto runs it alongside the arm with `awaitAll`, which is how a real scoring routine gets built. Watch `DriveToPose/DistanceError` while it runs. "Score Preload at Nearest Tag" goes one step further: drive and raise the arm together, eject, then stow.

[Back to the README](../README.md)
