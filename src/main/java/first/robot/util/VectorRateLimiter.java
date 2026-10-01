package first.robot.util;

import java.util.function.DoubleSupplier;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.system.Timer;

/**
 * Limits how fast a 2D velocity can change, so a joystick slammed forward ramps up instead of
 * stepping. That softens current spikes, wheel slip, and tipping, without weakening the robot once
 * it's moving or pushing.
 *
 * <p>It works on the whole vector, not each axis. Limiting x and y separately bends diagonal moves:
 * the slower-changing axis lags, so the robot curves. Here the velocity moves straight toward the
 * target, as fast as the limit allows.
 *
 * <p>Slowing down gets its own, usually higher, limit so the driver can always stop quickly.
 */
public class VectorRateLimiter {
  private final DoubleSupplier maxAccel;
  private final DoubleSupplier maxDecel;
  private final DoubleSupplier clock;
  private Translation2d last = Translation2d.ZERO;
  private double lastTime;

  /**
   * @param maxAccel how fast speed may increase, in units/s per second
   * @param maxDecel how fast speed may decrease, in units/s per second
   */
  public VectorRateLimiter(DoubleSupplier maxAccel, DoubleSupplier maxDecel) {
    // AdvantageKit fixes Timer.getTimestamp() for each robot loop, so replays reproduce this
    // exactly.
    this(maxAccel, maxDecel, Timer::getTimestamp);
  }

  /** Lets tests supply their own clock. */
  VectorRateLimiter(DoubleSupplier maxAccel, DoubleSupplier maxDecel, DoubleSupplier clock) {
    this.maxAccel = maxAccel;
    this.maxDecel = maxDecel;
    this.clock = clock;
    lastTime = clock.getAsDouble();
  }

  /** Returns the input, moved no further from the last output than the limits allow. */
  public Translation2d calculate(Translation2d input) {
    double now = clock.getAsDouble();
    double dt = now - lastTime;
    lastTime = now;

    var change = input.minus(last);
    // Braking is any change that points against the current motion, including the first half of a
    // reversal
    boolean slowingDown = change.getX() * last.getX() + change.getY() * last.getY() < 0;
    double maxChange = (slowingDown ? maxDecel : maxAccel).getAsDouble() * dt;
    if (change.getNorm() > maxChange) {
      change = change.times(maxChange / change.getNorm());
    }
    last = last.plus(change);
    return last;
  }

  /** Starts from {@code value}, such as the robot's current velocity when a command begins. */
  public void reset(Translation2d value) {
    last = value;
    lastTime = clock.getAsDouble();
  }
}
