package first.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.util.MathUtil;

/** The arithmetic behind DriveCharacterization.findEncoderOffsets. */
class EncoderOffsetsTest {
  @Test
  void newOffsetsMakeTheCurrentReadingsZero() {
    var readings =
        new Rotation2d[] {
          Rotation2d.fromRotations(0.1),
          Rotation2d.fromRotations(-0.3),
          Rotation2d.fromRotations(0.45),
          Rotation2d.fromRotations(0.0)
        };
    double[] offsets = DriveCharacterization.encoderOffsets(readings);

    for (int i = 0; i < readings.length; i++) {
      // The reading with the new offset instead of the old one: raw + new offset.
      double raw = readings[i].getRotations() - DriveConstants.MODULES[i].encoderOffsetRotations();
      assertEquals(0.0, MathUtil.inputModulus(raw + offsets[i], -0.5, 0.5), 1e-9);
      // Always within the range the CANcoder accepts.
      assertEquals(true, offsets[i] >= -0.5 && offsets[i] < 0.5);
    }
  }
}
