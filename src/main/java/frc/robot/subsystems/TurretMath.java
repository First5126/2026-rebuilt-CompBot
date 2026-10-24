package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import frc.robot.constants.TurretConstants;

final class TurretMath {
  private TurretMath() {}

  static double clampMechanismDegrees(double requestedDegrees, double dynamicOffsetDegrees) {
    double minDegrees = TurretConstants.MIN_ANGLE.in(Degrees);
    double maxDegrees = TurretConstants.MAX_ANGLE.in(Degrees);
    double requestedWithOffsetDegrees = requestedDegrees + dynamicOffsetDegrees;
    return Math.max(minDegrees, Math.min(requestedWithOffsetDegrees, maxDegrees));
  }

  static double mechanismToMotorDegrees(double mechanismDegrees) {
    return mechanismDegrees * TurretConstants.GEAR_RATIO;
  }

  static double computeClampedMotorDegrees(double requestedDegrees, double dynamicOffsetDegrees) {
    return mechanismToMotorDegrees(clampMechanismDegrees(requestedDegrees, dynamicOffsetDegrees));
  }
}
