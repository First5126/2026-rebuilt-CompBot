package frc.robot.constants;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;

import java.util.Map;

public class ShootingMechanismConstants {
  public static final Angle turretMaximumError = Degree.of(5);
  public static final Angle hoodMaximumError = Degree.of(2);
  public static final AngularVelocity flyWheelMaximumError = RotationsPerSecond.of(2);
  public static final Time mechanismDelay = Milliseconds.of(52.647);
  //public static final Time mechanismDelay = Milliseconds.of(1000);

  // Distance first then Time
  public static final InterpolatingDoubleTreeMap DISTANCE_TO_TIME_INTERPOLATOR =
      InterpolatingDoubleTreeMap.ofEntries(
          Map.entry(1.8, 0.8),
          Map.entry(2.12, 1.0),
          Map.entry(2.74, 1.0),
          Map.entry(3.1, 1.01),
          Map.entry(3.46, 1.04),
          Map.entry(4.11, 1.05),
          Map.entry(2.41, 0.93),
          Map.entry(4.23, 1.09),
          Map.entry(5.6, 1.28),
          Map.entry(5.25, 1.22),
          Map.entry(4.925, 1.13));
}
