package frc.robot.constants;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public class ShootingMechanismConstants {
  public static final Angle turretMaximumError = Degree.of(5);
  public static final Angle hoodMaximumError = Degree.of(2);
  public static final AngularVelocity flyWheelMaximumError = RotationsPerSecond.of(2);
}
