package frc.robot.constants;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Distance;
import java.util.Map;

public class FlyWheelConstants {
  public static double kP = 0.6;
  public static double kI = 0;
  public static double kD = 0.003;
  public static double kS = 0.32;
  public static double kV = 0.1168;
  public static double kA = 0;

  public static final int gearRatio = 2;
  public static final Distance radius = Meters.of(0.0508);

  public static final InterpolatingDoubleTreeMap DISTANCE_TO_SPEED_INTERPOLATOR =
      InterpolatingDoubleTreeMap.ofEntries(
          Map.entry(1.8, 42.5),
          Map.entry(2.12, 45.0),
          
          Map.entry(2.74, 49.0),
          Map.entry(3.12, 52.0),
          Map.entry(3.68, 52.0),
          Map.entry(4.32, 54.0));
}
