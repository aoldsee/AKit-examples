# The battery and brownouts

Why robots brown out, how the simulator models it, and the two limits this code uses to prevent it.

A brownout is when the battery voltage drops so low the robot controller can't keep everything running. On SystemCore that line is 6.75 V, and the robot stays browned out until the voltage climbs back above 7.25 V.

The cause is almost always current. A battery behaves like a voltage source with a small resistor inside it (about 0.02 ohms, counting wires and connectors). Every amp the robot draws drops the voltage by amps times that resistance, so 300 A costs about 6 V. That resistor also means a battery has a maximum power it can ever deliver, about 1800-2000 W for FRC batteries. Four drive motors launching hard with no supply current limit can ask for several times that.

The simulator models this. Each mechanism's sim model reports how much current it's drawing, `SimWorld` adds them up and works out the sagging voltage, and every simulated motor then runs on that lower voltage. Graph `RealOutputs/Sim/Battery/Volts` next to `RealOutputs/Sim/Battery/CurrentAmps/Drive` in AdvantageScope and floor it from a standstill to see it happen. If the voltage dips below 6.75 V, the "Simulated brownout" alert fires. The robot's weight matters here, so set `ROBOT_MASS_KG` in `Constants.java` to the real robot's.

## Supply current limits

The main fix is a **supply current limit**: a cap on how much current each motor may pull from the battery. `DriveConstants.DRIVE_SUPPLY_CURRENT_LIMIT_AMPS` sets it for the drive motors.

- It's different from a **stator** current limit, which caps the current inside the motor itself. That's about how hard the motor pushes (torque) and how hot it gets; the supply limit is about the battery.
- A motor draws much less from the battery than it does inside itself at low speed. The motor controller only connects the battery for part of the time (the fraction is applied volts / battery volts), so battery current is roughly motor current times that fraction. Pushing hard while barely moving needs only a few volts, so it takes a small share of the battery. It isn't zero, though: pushing at the 150 A stator limit while standing still, a drive motor would pull about 47 A from the battery, all of it turned into heat. That's over the 40 A supply limit, so with this robot's numbers the supply limit is the one in charge from a standstill, holding the motor to about 139 A.
- So near a standstill the supply limit only trims the push a little (139 A instead of 150 A). It does most of its work at speed, where the battery is actually hurting, and the robot still launches and pushes hard.

## Acceleration (slew) limits

The joystick drive commands also limit how quickly the robot's velocity can change ("slew rate" means rate of change; see `util/VectorRateLimiter.java`). It's a blunter tool for brownouts than a supply limit, because it softens every start, even slow ones that cost the battery almost nothing. It earns its place by catching what a supply limit doesn't:

- the instant current spike when the stick is slammed (the supply limit takes a moment to react)
- reversals from full forward to full back
- wheel slip and tipping
- shock on the gearboxes

Driving also just feels smoother. Two details:

- The limit applies to the whole velocity vector, not x and y separately. Limiting each axis on its own makes diagonal moves curve.
- Slowing down has its own, higher limit, so the driver can always stop quickly. Any change that pushes against the current motion counts as slowing down: a reversal until the robot passes through zero, and a sharp turn at the same speed.

[Back to the README](../README.md)
