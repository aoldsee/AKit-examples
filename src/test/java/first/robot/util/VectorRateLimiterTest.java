package first.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Translation2d;

/** Uses a fake clock, so time only moves when the test says. */
class VectorRateLimiterTest {
  private static final double ACCEL = 10.0;
  private static final double DECEL = 15.0;
  private static final double LOOP = 0.02;

  private double time = 0.0;
  private VectorRateLimiter limiter;

  @BeforeEach
  void setup() {
    time = 0.0;
    limiter = new VectorRateLimiter(() -> ACCEL, () -> DECEL, () -> time);
  }

  private Translation2d step(Translation2d input) {
    time += LOOP;
    return limiter.calculate(input);
  }

  @Test
  void slammingTheStickRampsUp() {
    var output = step(new Translation2d(4.0, 0.0));
    // 10 m/s^2 for one 20 ms loop.
    assertEquals(0.2, output.getX(), 1e-9);
    assertEquals(0.0, output.getY(), 1e-9);
  }

  @Test
  void reachesTheTargetAndStaysThere() {
    Translation2d output = Translation2d.ZERO;
    for (int i = 0; i < 100; i++) {
      output = step(new Translation2d(4.0, 0.0));
    }
    assertEquals(4.0, output.getX(), 1e-9);
  }

  @Test
  void diagonalsStayStraight() {
    // Limiting x and y separately would give (0.2, 0.2) here, pointing 45 degrees, and the robot
    // would curve toward the 3-4-5 direction as the x axis caught up.
    var output = step(new Translation2d(3.0, 4.0));
    assertEquals(0.2, output.getNorm(), 1e-9);
    assertEquals(Math.atan2(4.0, 3.0), Math.atan2(output.getY(), output.getX()), 1e-9);
  }

  @Test
  void stoppingUsesTheDecelerationLimit() {
    limiter.reset(new Translation2d(4.0, 0.0));
    var output = step(Translation2d.ZERO);
    assertEquals(4.0 - DECEL * LOOP, output.getX(), 1e-9);
  }

  @Test
  void reversingBrakesFirstThenAccelerates() {
    limiter.reset(new Translation2d(0.1, 0.0));
    // Still moving forward, so the change toward reverse is braking.
    var braking = step(new Translation2d(-4.0, 0.0));
    assertEquals(0.1 - DECEL * LOOP, braking.getX(), 1e-9);
    // Now moving backward, so speeding up in reverse uses the acceleration limit.
    var accelerating = step(new Translation2d(-4.0, 0.0));
    assertEquals(braking.getX() - ACCEL * LOOP, accelerating.getX(), 1e-9);
  }
}
