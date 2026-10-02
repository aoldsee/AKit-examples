package first.robot.subsystems.drive;

import static first.robot.subsystems.drive.DriveCommands.linearVelocityFromJoysticks;
import static first.robot.subsystems.drive.DriveCommands.rotationFromJoystick;
import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Translation2d;

/** Plain math, no robot needed. These run in milliseconds. */
class JoystickShapingTest {
  @Test
  void smallStickMovementsAreIgnored() {
    assertEquals(Translation2d.ZERO, linearVelocityFromJoysticks(0.05, -0.05));
    assertEquals(0.0, rotationFromJoystick(0.09));
  }

  @Test
  void fullStickIsFullSpeed() {
    assertEquals(1.0, linearVelocityFromJoysticks(1.0, 0.0).getNorm(), 1e-9);
    assertEquals(1.0, rotationFromJoystick(1.0), 1e-9);
    assertEquals(-1.0, rotationFromJoystick(-1.0), 1e-9);
  }

  @Test
  void cornerOfTheStickIsStillFullSpeed() {
    // (1, 1) has a magnitude of 1.41; without the clamp this would ask for 141% speed.
    var velocity = linearVelocityFromJoysticks(1.0, 1.0);
    assertEquals(1.0, velocity.getNorm(), 1e-9);
    assertEquals(Math.PI / 4, velocity.getAngle().orElseThrow().getRadians(), 1e-9);
  }

  @Test
  void halfStickIsMuchLessThanHalfSpeed() {
    // Deadband rescales 0.5 to (0.5 - 0.1) / 0.9, then squaring shrinks it further. That's what
    // gives fine control near the center.
    double expected = Math.pow((0.5 - 0.1) / 0.9, 2);
    assertEquals(expected, linearVelocityFromJoysticks(0.5, 0.0).getNorm(), 1e-9);
    assertEquals(-expected, rotationFromJoystick(-0.5), 1e-9);
  }

  @Test
  void directionIsPreserved() {
    var velocity = linearVelocityFromJoysticks(-0.3, 0.6);
    assertEquals(Math.atan2(0.6, -0.3), Math.atan2(velocity.getY(), velocity.getX()), 1e-9);
  }
}
