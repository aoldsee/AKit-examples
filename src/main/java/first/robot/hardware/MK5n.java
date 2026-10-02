package first.robot.hardware;

import org.wpilib.math.util.Units;

/** SDS MK5n swerve module as built */
public final class MK5n {
  private MK5n() {}

  /** Drive motor rotations per wheel rotation, R1 gearing. */
  public static final double DRIVE_RATIO_R1 = 7.03;

  /** Drive motor rotations per wheel rotation, R2 gearing. */
  public static final double DRIVE_RATIO_R2 = 6.03;

  /** Drive motor rotations per wheel rotation, R3 gearing. */
  public static final double DRIVE_RATIO_R3 = 5.27;

  /** Steer motor rotations per module rotation. */
  public static final double STEER_RATIO = 287.0 / 11.0;

  /**
   * Drive motor rotations per module rotation. The drive gear meshes with a bevel on the steering
   * axis, so turning the module also turns the drive motor even when the wheel doesn't roll.
   */
  public static final double COUPLING_RATIO = 54.0 / 16.0;

  public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(4.0);
  public static final double WHEEL_RADIUS_METERS = WHEEL_DIAMETER_METERS / 2.0;
  public static final double WHEEL_WIDTH_METERS = Units.inchesToMeters(2.5);

  /** Molded spike tread on standard field carpet. */
  public static final double WHEEL_MOLDED_COF_CARPET = 2.255;

  /** Neoprene tread on standard field carpet. */
  public static final double WHEEL_NEOPRENE_COF_CARPET = 1.2;

  /** Colson wheel on standard field carpet. */
  public static final double WHEEL_COLSON_COF_CARPET = 1.1;
}
