package first.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.DoubleSupplier;
import org.wpilib.system.Timer;
import org.wpilib.units.measure.Angle;

/**
 * Samples odometry signals faster than the 50 Hz main loop and buffers them in queues that {@link
 * Drive} drains each cycle.
 *
 * <p>Lock order is signalsLock, then {@link Drive#odometryLock}. Drive holds odometryLock while it
 * reads inputs, so the queues and the timestamps written alongside them always stay the same
 * length.
 */
public final class PhoenixOdometryThread extends Thread {
  // Holds about 80 ms at 250 Hz. If the main loop stalls longer than that, new samples are
  // dropped. Harmless: each sample is a total position, not a change, so the next one still
  // lands in the right place.
  private static final int QUEUE_CAPACITY = 20;

  private static final boolean IS_CAN_FD = DriveConstants.CAN_BUS.isNetworkFD();
  private static PhoenixOdometryThread instance = null;

  private final Lock signalsLock = new ReentrantLock();
  private final List<BaseStatusSignal> phoenixSignals = new ArrayList<>();
  private final List<DoubleSupplier> genericSignals = new ArrayList<>();
  private final List<Queue<Double>> phoenixQueues = new ArrayList<>();
  private final List<Queue<Double>> genericQueues = new ArrayList<>();
  private final List<Queue<Double>> timestampQueues = new ArrayList<>();

  public static PhoenixOdometryThread getInstance() {
    if (instance == null) {
      instance = new PhoenixOdometryThread();
    }
    return instance;
  }

  private PhoenixOdometryThread() {
    setName("PhoenixOdometryThread");
    setDaemon(true);
  }

  /** No-op when nothing registered, which is the case in replay. */
  @Override
  public void start() {
    if (!timestampQueues.isEmpty()) {
      super.start();
    }
  }

  /**
   * Pass a clone of the signal. Refreshing it here and on the main thread at once would race on its
   * cached value.
   */
  public Queue<Double> registerSignal(StatusSignal<Angle> signal) {
    Queue<Double> queue = new ArrayBlockingQueue<>(QUEUE_CAPACITY);
    signalsLock.lock();
    Drive.odometryLock.lock();
    try {
      phoenixSignals.add(signal);
      phoenixQueues.add(queue);
    } finally {
      signalsLock.unlock();
      Drive.odometryLock.unlock();
    }
    return queue;
  }

  /** For non-Phoenix sources sampled at the same instants as the Phoenix signals. */
  public Queue<Double> registerSignal(DoubleSupplier signal) {
    Queue<Double> queue = new ArrayBlockingQueue<>(QUEUE_CAPACITY);
    signalsLock.lock();
    Drive.odometryLock.lock();
    try {
      genericSignals.add(signal);
      genericQueues.add(queue);
    } finally {
      signalsLock.unlock();
      Drive.odometryLock.unlock();
    }
    return queue;
  }

  /** Each consumer needs its own timestamp queue because draining one would starve the others. */
  public Queue<Double> makeTimestampQueue() {
    Queue<Double> queue = new ArrayBlockingQueue<>(QUEUE_CAPACITY);
    Drive.odometryLock.lock();
    try {
      timestampQueues.add(queue);
    } finally {
      Drive.odometryLock.unlock();
    }
    return queue;
  }

  @Override
  public void run() {
    while (true) {
      signalsLock.lock();
      try {
        if (IS_CAN_FD && !phoenixSignals.isEmpty()) {
          // Blocks until every signal has a new frame, so one sample holds values from the same
          // instant. The 2x period timeout keeps one dead device from stalling all odometry.
          BaseStatusSignal.waitForAll(2.0 / DriveConstants.ODOMETRY_FREQUENCY, phoenixSignals);
        } else {
          // Phoenix rejects waitForAll on multiple signals over CAN 2.0, Pro license or not.
          Thread.sleep((long) (1000.0 / DriveConstants.ODOMETRY_FREQUENCY));
          if (!phoenixSignals.isEmpty()) {
            BaseStatusSignal.refreshAll(phoenixSignals);
          }
        }
      } catch (InterruptedException e) {
        e.printStackTrace();
      } finally {
        signalsLock.unlock();
      }

      Drive.odometryLock.lock();
      try {
        // Back-date the sample by the average CAN latency so it lines up with when the frames
        // were measured, not when this thread woke up.
        double timestamp = Timer.getMonotonicTimestamp();
        if (!phoenixSignals.isEmpty()) {
          double totalLatency = 0.0;
          for (BaseStatusSignal signal : phoenixSignals) {
            totalLatency += signal.getTimestamp().getLatency();
          }
          timestamp -= totalLatency / phoenixSignals.size();
        }

        for (int i = 0; i < phoenixSignals.size(); i++) {
          phoenixQueues.get(i).offer(phoenixSignals.get(i).getValueAsDouble());
        }
        for (int i = 0; i < genericSignals.size(); i++) {
          genericQueues.get(i).offer(genericSignals.get(i).getAsDouble());
        }
        for (Queue<Double> queue : timestampQueues) {
          queue.offer(timestamp);
        }
      } finally {
        Drive.odometryLock.unlock();
      }
    }
  }
}
