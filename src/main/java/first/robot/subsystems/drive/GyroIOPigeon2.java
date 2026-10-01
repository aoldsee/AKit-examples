package first.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import first.robot.generated.TunerConstants;
import java.util.Queue;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.util.Units;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;

public class GyroIOPigeon2 implements GyroIO {
  // Protected so the sim subclass can hand it to its sim model.
  protected final Pigeon2 pigeon =
      new Pigeon2(TunerConstants.DrivetrainConstants.Pigeon2Id, TunerConstants.kCANBus);
  private final StatusSignal<Angle> yaw;
  private final StatusSignal<AngularVelocity> yawVelocity;
  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;

  public GyroIOPigeon2() {
    var config = TunerConstants.DrivetrainConstants.Pigeon2Configs;
    pigeon.getConfigurator().apply(config != null ? config : new Pigeon2Configuration());
    pigeon.getConfigurator().setYaw(0.0);

    // false skips the refresh Phoenix normally does on lookup. Right after startup the first status
    // frame may not have arrived, and that refresh would log a CAN error. updateInputs refreshes
    // every loop anyway.
    yaw = pigeon.getYaw(false);
    yawVelocity = pigeon.getAngularVelocityZWorld(false);
    yaw.setUpdateFrequency(DriveConstants.ODOMETRY_FREQUENCY);
    yawVelocity.setUpdateFrequency(50.0);
    pigeon.optimizeBusUtilization();
    yawTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
    // The thread refreshes its own copy so it never races the main loop's refresh of yaw.
    yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(yaw.clone());
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = BaseStatusSignal.refreshAll(yaw, yawVelocity).isOK();
    inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble());

    inputs.odometryYawTimestamps = yawTimestampQueue.stream().mapToDouble(v -> v).toArray();
    inputs.odometryYawPositions =
        yawPositionQueue.stream().map(Rotation2d::fromDegrees).toArray(Rotation2d[]::new);
    yawTimestampQueue.clear();
    yawPositionQueue.clear();
  }
}
