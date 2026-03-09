package frc.robot.constants;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;

public class ShootingMechanismConstants {
  public static final Angle turretMaximumError = Degree.of(5);
  public static final Angle hoodMaximumError = Degree.of(1);
  public static final LinearAcceleration gravity = MetersPerSecondPerSecond.of(9.8);

  // TODO: get real angle readings
  public static final Angle maximumDistanceHoodAngle = Degrees.of(45);

  public static final LinearVelocity maximumFlywheelSpeed = MetersPerSecond.of(11.969);
}
