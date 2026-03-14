package frc.robot.constants;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Distance;
import java.util.Map;

public class FlyWheelConstants {
  public static double kP = 0.048188;
  public static double kI = 0;
  public static double kD = 0.003;
  public static double kS = 0.28927;
  public static double kV = 0.12936;
  public static double kA = 0.0068105;

  public static final int gearRatio = 2;
  public static final Distance radius = Meters.of(0.0508);

  public static final InterpolatingDoubleTreeMap DISTANCE_TO_SPEED_INTERPOLATOR =
      InterpolatingDoubleTreeMap.ofEntries(
          Map.entry(1.8, 42.5),
          Map.entry(2.12, 45.0),
          Map.entry(2.74, 49.0),
          Map.entry(3.1, 50.5),
          Map.entry(3.46, 52.0),
          Map.entry(4.11, 53.5),
          Map.entry(2.41, 47.5),
          Map.entry(4.23, 54.75),
          Map.entry(5.6, 59.0),
          Map.entry(5.25, 58.3),
          Map.entry(4.925, 57.25));

  public static final InterpolatingDoubleTreeMap MIDDLE_DISTANCE_TO_SPEED_INTERPOLATOR =
      InterpolatingDoubleTreeMap.ofEntries(
          Map.entry(1.8, 42.5),
          Map.entry(2.12, 45.0),
          Map.entry(2.74, 49.0),
          Map.entry(3.1, 50.5),
          Map.entry(3.46, 52.0),
          Map.entry(4.11, 53.5),
          Map.entry(2.41, 47.5),
          Map.entry(4.23, 54.75),
          Map.entry(5.6, 59.0),
          Map.entry(5.25, 58.3),
          Map.entry(4.925, 57.25));
}
