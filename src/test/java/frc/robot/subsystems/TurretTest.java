package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;

import frc.robot.constants.TurretConstants;
import org.junit.jupiter.api.Test;

class TurretTest {

  @Test
  void clampMechanismDegrees_clampsBelowMin() {
    double result = TurretMath.clampMechanismDegrees(-1000.0, 0.0);

    assertEquals(TurretConstants.MIN_ANGLE.in(edu.wpi.first.units.Units.Degrees), result, 1e-9);
  }

  @Test
  void clampMechanismDegrees_clampsAboveMax() {
    double result = TurretMath.clampMechanismDegrees(1000.0, 0.0);

    assertEquals(TurretConstants.MAX_ANGLE.in(edu.wpi.first.units.Units.Degrees), result, 1e-9);
  }

  @Test
  void clampMechanismDegrees_appliesOffsetBeforeClamp() {
    double nearMax = TurretConstants.MAX_ANGLE.in(edu.wpi.first.units.Units.Degrees) - 1.0;
    double result = TurretMath.clampMechanismDegrees(nearMax, 5.0);

    assertEquals(TurretConstants.MAX_ANGLE.in(edu.wpi.first.units.Units.Degrees), result, 1e-9);
  }

  @Test
  void mechanismToMotorDegrees_scalesByGearRatio() {
    double result = TurretMath.mechanismToMotorDegrees(12.5);

    assertEquals(12.5 * TurretConstants.GEAR_RATIO, result, 1e-9);
  }

  @Test
  void computeClampedMotorDegrees_clampsThenConverts() {
    double result = TurretMath.computeClampedMotorDegrees(1000.0, 0.0);

    assertEquals(
        TurretConstants.MAX_ANGLE.in(edu.wpi.first.units.Units.Degrees)
            * TurretConstants.GEAR_RATIO,
        result,
        1e-9);
  }
}
