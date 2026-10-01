package first.robot.subsystems.drive;

/**
 * Simulated gyro. Reads exactly like {@link GyroIOPigeon2}; {@link SwerveDriveSim} sets the
 * simulated Pigeon's yaw from how the modules are actually moving.
 */
public class GyroIOPigeon2Sim extends GyroIOPigeon2 {
  public GyroIOPigeon2Sim(SwerveDriveSim driveSim) {
    super();
    driveSim.setGyro(pigeon.getSimState());
  }
}
