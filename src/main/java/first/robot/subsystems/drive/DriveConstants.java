package first.robot.subsystems.drive;

import static org.wpilib.units.Units.MetersPerSecond;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import first.robot.generated.TunerConstants;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N3;
import org.wpilib.math.util.Units;

/**
 * Values derived from {@link TunerConstants}. Keeping them here lets TunerConstants stay a drop-in
 * replacement for generator output.
 */
public final class DriveConstants {
  private DriveConstants() {}

  /** Module constants in FL, FR, BL, BR order. Every module-indexed array uses this order. */
  @SuppressWarnings("unchecked")
  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>[]
      MODULE_CONSTANTS =
          new SwerveModuleConstants[] {
            TunerConstants.FrontLeft,
            TunerConstants.FrontRight,
            TunerConstants.BackLeft,
            TunerConstants.BackRight
          };

  /** Hz. CAN 2.0 can't carry 250 Hz of drive and steer signals for four modules. */
  public static final double ODOMETRY_FREQUENCY =
      TunerConstants.kCANBus.isNetworkFD() ? 250.0 : 100.0;

  /**
   * How much the pose estimator trusts odometry, as standard deviations (meters, meters, radians).
   *
   * <p>Each vision frame moves the estimate odometry / (odometry + vision) of the way toward the
   * camera's answer. That fraction is the same however often frames arrive, so it has to be picked
   * together with the camera's frame rate:
   *
   * <ul>
   *   <li>Too large a fraction, and the pose jumps with each frame's noise. With WPILib's default
   *       of 0.1 m, a typical frame here (5 tags at 4 m, about 1 cm) moves the estimate over 90% of
   *       the way.
   *   <li>Too small, and a real error (a bump, a wheel slipping) takes a long time to fix.
   * </ul>
   *
   * <p>At 0.002 m a typical frame moves it about 20% of the way. At 90 frames a second that still
   * fixes a real error in well under a second, while averaging out most of the noise. A camera
   * running at 30 fps would need about 3 times this value to correct as quickly.
   */
  public static final Matrix<N3, N1> ODOMETRY_STD_DEVS = VecBuilder.fill(0.002, 0.002, 0.002);

  /** Robot-relative module positions, meters. */
  public static final Translation2d[] MODULE_TRANSLATIONS = moduleTranslations();

  /** Meters, center to the farthest module. */
  public static final double DRIVE_BASE_RADIUS = driveBaseRadius();

  public static final double MAX_LINEAR_SPEED = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

  // Default driver acceleration limits, m/s^2. Tunable at runtime under /Tuning/Drive/. Liberal on
  // purpose: they only clip sudden stick slams. Deceleration is higher so the driver can always
  // stop quickly.
  public static final double DRIVER_MAX_ACCEL = 10.0;
  public static final double DRIVER_MAX_DECEL = 15.0;

  // Driver acceleration and deceleration cap while the arm is away from its hard stops, m/s^2.
  // A hard start or stop swings a raised arm, which can knock it off its goal and, on a tall
  // robot, help tip it. This applies to slowing down too, so stops take longer with the arm up.
  public static final double ARM_RAISED_MAX_ACCEL = 4.0;

  // DriveToPose limits and tolerances. Gentler than the driver's limits: it's for lining up, not
  // racing, and a smooth arrival settles faster than an aggressive one.
  public static final double ALIGN_MAX_VELOCITY = 3.0; // m/s
  public static final double ALIGN_MAX_ACCEL = 4.0; // m/s^2
  public static final double ALIGN_MAX_ANGULAR_VELOCITY = 6.0; // rad/s
  public static final double ALIGN_MAX_ANGULAR_ACCEL = 12.0; // rad/s^2
  public static final double ALIGN_DISTANCE_TOLERANCE = 0.02; // m
  public static final double ALIGN_ANGLE_TOLERANCE = Math.toRadians(2.0);
  // Also has to be nearly stopped (see AlignTargets.isAligned).
  public static final double ALIGN_SETTLED_SPEED = 0.05; // m/s

  /**
   * Front to back over the bumpers, meters. A square 27.5 in frame (modules 11.5 in from center)
   * plus about 3.5 in of bumper on each end. Measure the real robot.
   */
  public static final double BUMPER_LENGTH_METERS = Units.inchesToMeters(35.0);

  /** Rad/s, assuming all wheels drive tangentially at max linear speed. */
  public static final double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS;

  private static Translation2d[] moduleTranslations() {
    var translations = new Translation2d[MODULE_CONSTANTS.length];
    for (int i = 0; i < MODULE_CONSTANTS.length; i++) {
      translations[i] =
          new Translation2d(MODULE_CONSTANTS[i].LocationX, MODULE_CONSTANTS[i].LocationY);
    }
    return translations;
  }

  private static double driveBaseRadius() {
    double radius = 0.0;
    for (var translation : MODULE_TRANSLATIONS) {
      radius = Math.max(radius, translation.getNorm());
    }
    return radius;
  }
}
