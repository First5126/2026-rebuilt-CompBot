package frc.robot.constants;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import java.util.Map;

public class HoodConstants {
  public static double kP = 135.0;
  public static double kI = 0.0;
  public static double kD = 0.0;
  public static double kS = 0.35;
  public static double kV = 13.84;

  // TODO: Very important to fix this so we dont rip the hood off
  public static final InterpolatingDoubleTreeMap DISTANCE_TO_ANGLE_INTERPOLATOR =
      InterpolatingDoubleTreeMap.ofEntries(
          Map.entry(1.8, 0.0),
          Map.entry(2.12, 0.0),
          Map.entry(2.74, 1.0),
          Map.entry(3.1, 10.0),
          Map.entry(3.68, 10.0),
          Map.entry(4.32, 10.0));
}
