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

  public static class InterpolationSet {
    public final InterpolatingDoubleTreeMap distanceToTimeOfFlight;
    public final InterpolatingDoubleTreeMap distanceToFlyWheelSpeed;
    public final InterpolatingDoubleTreeMap distanceToHoodAngle;

    public InterpolationSet(
        InterpolatingDoubleTreeMap distanceToTimeOfFlight,
        InterpolatingDoubleTreeMap distanceToFlyWheelSpeed,
        InterpolatingDoubleTreeMap distanceToHoodAngle) {

      this.distanceToTimeOfFlight = distanceToTimeOfFlight;
      this.distanceToFlyWheelSpeed = distanceToFlyWheelSpeed;
      this.distanceToHoodAngle = distanceToHoodAngle;
    }
  }

  public static final Angle turretMaximumError = Degree.of(2.5);
  public static final Angle hoodMaximumError = Degree.of(2);
  public static final AngularVelocity flyWheelMaximumError = RotationsPerSecond.of(2);
  public static final Time mechanismDelay = Milliseconds.of(52.647);
  public static final Time computationDelay = Milliseconds.of(20);
  public static final int updateCounter = 2;

  // Distance first then Time
  private static final InterpolatingDoubleTreeMap DISTANCE_TO_TIME_INTERPOLATOR_HUB =
      InterpolatingDoubleTreeMap.ofEntries(
          Map.entry(1.8, 0.8),
          Map.entry(2.12, 1.0),
          Map.entry(2.74, 1.0),
          Map.entry(3.1, 1.01),
          Map.entry(3.46, 1.04),
          Map.entry(4.11, 1.05),
          Map.entry(2.41, 0.93),
          Map.entry(4.23, 1.09),
          Map.entry(5.788, 1.5),
          Map.entry(5.619, 1.57),
          Map.entry(5.418, 1.47),
          Map.entry(5.150, 1.36),
          Map.entry(5.0, 1.31),
          Map.entry(4.394, 1.26));

  private static final InterpolatingDoubleTreeMap DISTANCE_TO_SPEED_INTERPOLATOR_HUB =
      InterpolatingDoubleTreeMap.ofEntries(
          Map.entry(1.8, 42.5),
          Map.entry(2.12, 45.0),
          Map.entry(2.74, 49.0),
          Map.entry(3.1, 50.5),
          Map.entry(3.46, 53.0),
          Map.entry(4.11, 58.5),
          Map.entry(2.41, 47.5),
          Map.entry(4.23, 58.0),
          Map.entry(5.788, 62.0),
          Map.entry(5.619, 60.0),
          Map.entry(5.418, 59.0),
          Map.entry(5.150, 59.0),
          Map.entry(5.0, 58.5),
          Map.entry(4.75, 58.25), // Added an extra point to fill in a dead area
          Map.entry(4.394, 57.5));

  private static final InterpolatingDoubleTreeMap DISTANCE_TO_ANGLE_INTERPOLATOR_HUB =
      InterpolatingDoubleTreeMap.ofEntries(
          Map.entry(1.8, 0.0),
          Map.entry(2.12, 0.0),
          Map.entry(2.74, 0.0),
          Map.entry(3.1, 1.5),
          Map.entry(3.46, 4.0),
          Map.entry(4.11, 6.0),
          Map.entry(2.41, 0.0),
          Map.entry(4.23, 6.5),
          Map.entry(5.788, 5.95),
          Map.entry(5.619, 5.85),
          Map.entry(5.418, 5.75),
          Map.entry(5.150, 5.75),
          Map.entry(5.0, 5.5),
          Map.entry(4.394, 5.5));

  private static final InterpolatingDoubleTreeMap DISTANCE_TO_TIME_INTERPOLATOR_FLOOR =
      InterpolatingDoubleTreeMap.ofEntries(
          Map.entry(2.3, 1.33),
          Map.entry(3.75, 1.6),
          Map.entry(4.3, 1.52),
          Map.entry(5.1, 1.46),
          Map.entry(5.65, 1.4),
          Map.entry(6.0, 1.35),
          Map.entry(7.3, 1.45),
          Map.entry(8.5, 1.52),
          // Fudged from here
          Map.entry(16.0, 2.0));

  private static final InterpolatingDoubleTreeMap DISTANCE_TO_SPEED_INTERPOLATOR_FLOOR =
      InterpolatingDoubleTreeMap.ofEntries(
          Map.entry(2.3, 45.0),
          Map.entry(3.75, 55.0),
          Map.entry(4.3, 55.0),
          Map.entry(5.1, 55.0),
          Map.entry(5.65, 55.0),
          Map.entry(6.0, 55.0),
          Map.entry(7.3, 60.0),
          Map.entry(8.5, 65.0),
          // Fudged from here
          Map.entry(16.0, 85.0));

  private static final InterpolatingDoubleTreeMap DISTANCE_TO_ANGLE_INTERPOLATOR_FLOOR =
      InterpolatingDoubleTreeMap.ofEntries(
          Map.entry(2.3, 0.0),
          Map.entry(3.75, 0.0),
          Map.entry(4.3, 5.0),
          Map.entry(5.1, 10.0),
          Map.entry(5.65, 15.0),
          Map.entry(6.0, 20.0),
          Map.entry(7.3, 20.0),
          Map.entry(8.5, 20.0),
          // Fudged from here
          Map.entry(16.0, 25.0));

  public static final InterpolationSet floorInterpolation =
      new InterpolationSet(
          DISTANCE_TO_TIME_INTERPOLATOR_FLOOR,
          DISTANCE_TO_SPEED_INTERPOLATOR_FLOOR,
          DISTANCE_TO_ANGLE_INTERPOLATOR_FLOOR);
  public static final InterpolationSet hubInterpolation =
      new InterpolationSet(
          DISTANCE_TO_TIME_INTERPOLATOR_HUB,
          DISTANCE_TO_SPEED_INTERPOLATOR_HUB,
          DISTANCE_TO_ANGLE_INTERPOLATOR_HUB);
}
