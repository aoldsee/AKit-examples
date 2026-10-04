# Moving to a real robot

This code has never driven real hardware, so expect to work through these:

1. **Robot mass and size.** Weigh the robot with bumpers and battery and set `ROBOT_MASS_KG` in `Constants.java`. Measure front to back over the bumpers and set `BUMPER_LENGTH_METERS` in `DriveConstants.java`; the starting positions use it.
2. **Hardware specs.** `hardware/MK5n.java` assumes R2 gearing and `hardware/Motors.java` the Kraken X60 and X44. If the hardware differs, change them there; gear ratios, top speed, and feedforward gains are all derived from these.
3. **Team number.** Set the team's number in `.wpilib/wpilib_preferences.json` (it's 9999 here, for development).
4. **Swerve constants.** In `DriveConstants.java`, set each module's CAN IDs, inverts, and position in `MODULES`, plus the Pigeon's ID. Then find the encoder offsets:
   - Deploy, and leave the robot disabled.
   - Line every wheel up straight ahead by hand, with all the bevel gears facing the same side (a straightedge along each side helps).
   - Set `/Calibration/FindEncoderOffsets` to true from the dashboard. The console prints an offset for each module; paste them into `MODULES` and redeploy.
   - If the systems check later finds a wheel driving backward, that module was lined up 180° off: turn it around and calibrate again.
5. **Arm constants.** Set the CAN IDs, motor and encoder directions, and gear ratio in `ArmConstants.java`. The CANcoder offset must make the arm read **0 when it's horizontal**, because the gravity feedforward assumes that. To find it, hold the arm level (check with a level) and read `Arm/AbsolutePositionRad`. That reading is in radians and `ENCODER_OFFSET` is in rotations, so convert the reading first (divide by 2π), then subtract it from the current offset. For example, with `ENCODER_OFFSET = 0.10` and a level arm reading 0.35 rad: 0.35 / 2π = 0.056 rotations, so the new offset is 0.10 − 0.056 = 0.044. Redeploy and check the level arm now reads about 0.
6. **Intake constants.** Set the CAN ID, gear ratio, and beam break port in `IntakeConstants.java`. Then put a piece in by hand and watch `Intake/SensorBlocked`: if it reads false with the piece in, switch `BEAM_BREAK`.
7. **CAN buses.** The drivetrain is on SystemCore port `CAN_S0`, and the arm and intake on `CAN_S1`. Change them to match the wiring.
8. **Limelights.** Name them `limelight-front` and `limelight-back` in their web UI (or change `CAMERA_NAMES` in `VisionConstants.java`), measure where each camera sits on the robot and set `ROBOT_TO_CAMERA` in `VisionConstants.java`, and use an AprilTag pipeline (it publishes both the MegaTag1 and MegaTag2 results the code reads). The robot sends each camera its position at startup, overriding the web UI's camera pose. After deploying, check the web UI shows the same position; if the pitch is upside down, the sign conversion in `VisionIOLimelight.toLimelightCameraPose` is the place to look. Check the field layout in `FieldGeometry.FIELD` matches the current season.
9. **Field geometry.** The starting positions assume the three driver stations split the alliance wall evenly, with station 1 at the drivers' left. Check both against the game manual.
10. **Phoenix Pro.** This code uses Pro features (FOC commutation and fused CANcoders), which need a Phoenix Pro license on each device. Without one, the devices fall back to non-Pro behavior and raise an "unlicensed" alert.
11. **Run the Drive Systems Check** on blocks until it passes and every wheel points straight ahead.
12. **Characterize**, then tune.

## Pit check

**Drive Systems Check** in the Auto Choices list checks that every swerve module works. Put the robot on blocks (or give it a couple of meters of clear floor), select it, and enable in autonomous. It steers every module to 90° and back, then spins every wheel forward on the same voltage, and takes about 3 seconds. The result shows up as an alert: either "passed", or a list of what failed and what to check (a module that didn't steer, a wheel spinning backward, a dead motor, a wheel much slower than the rest).

It can't tell whether a module's encoder offset (the setting that says where "straight ahead" is) is wrong, so at the end, look: every wheel should point straight ahead. `DriveSystemsCheckTest` shows it catching each problem with deliberately broken fake modules.

[Back to the README](../README.md)
