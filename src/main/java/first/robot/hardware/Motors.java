package first.robot.hardware;

import org.wpilib.math.system.DCMotor;
import org.wpilib.math.util.Units;

/**
 * As-built specs for the motors on this robot, from CTRE's published curves (the same numbers
 * WPILib's DCMotor uses). Gains and simulation models are derived from these instead of being typed
 * in by hand, so changing a motor changes everything that depends on it.
 *
 * <p>FOC (field-oriented control, Phoenix Pro) trades a little top speed for noticeably more
 * torque, so it gets its own spec.
 */
public final class Motors {
  private Motors() {}

  /** Volts the specs are measured at. */
  public static final double NOMINAL_VOLTS = 12.0;

  /**
   * One motor's datasheet numbers, plus the values derived from them.
   *
   * @param stallTorqueNm torque with the shaft held still at 12 V
   * @param stallCurrentAmps current with the shaft held still at 12 V
   * @param freeCurrentAmps current spinning with no load
   * @param freeSpeedRpm speed with no load at 12 V
   */
  public record Spec(
      String name,
      double stallTorqueNm,
      double stallCurrentAmps,
      double freeCurrentAmps,
      double freeSpeedRpm) {
    /** Winding resistance. At stall there's no back-EMF, so all 12 V drops across it. */
    public double resistanceOhms() {
      return NOMINAL_VOLTS / stallCurrentAmps;
    }

    /** Torque per amp, N*m/A. */
    public double torquePerAmp() {
      return stallTorqueNm / stallCurrentAmps;
    }

    public double freeSpeedRps() {
      return freeSpeedRpm / 60.0;
    }

    /**
     * Speed per volt of back-EMF, rotations/s per volt. Slightly more than free speed / 12, because
     * even at free speed a little voltage is lost to resistance.
     */
    public double rpsPerVolt() {
      return freeSpeedRps() / (NOMINAL_VOLTS - resistanceOhms() * freeCurrentAmps);
    }

    /** WPILib's motor model for simulation and system identification math. */
    public DCMotor toDCMotor(int motorCount) {
      return new DCMotor(
          NOMINAL_VOLTS,
          stallTorqueNm,
          stallCurrentAmps,
          freeCurrentAmps,
          Units.rotationsPerMinuteToRadiansPerSecond(freeSpeedRpm),
          motorCount);
    }
  }

  public static final Spec KRAKEN_X60 = new Spec("Kraken X60", 7.09, 366, 2, 6000);
  public static final Spec KRAKEN_X60_FOC = new Spec("Kraken X60 (FOC)", 9.37, 483, 2, 5800);
  public static final Spec KRAKEN_X44 = new Spec("Kraken X44", 4.11, 279, 2, 7758);
  public static final Spec KRAKEN_X44_FOC = new Spec("Kraken X44 (FOC)", 5.01, 329, 2, 7368);
}
