package frc.robot.constants;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import java.util.Map;

public class HoodConstants {
  public static double kP = 135.0;
  public static double kI = 0.0;
  public static double kD = 0.0;
  public static double kS = 0.35;
  public static double kV = 13.84;

  public static double DUCK_SPEED = 0.25;

  // TODO: Very important to fix this so we dont rip the hood off
  public static final InterpolatingDoubleTreeMap DISTANCE_TO_ANGLE_INTERPOLATOR =
      InterpolatingDoubleTreeMap.ofEntries(
          Map.entry(1.8, 0.0),
          Map.entry(2.12, 0.0),
          Map.entry(2.74, 0.0),
          Map.entry(3.1, 1.5),
          Map.entry(3.46, 4.0),
          Map.entry(4.11, 6.0),
          Map.entry(2.41, 0.0),
          Map.entry(4.23, 6.5),
          Map.entry(5.6, 5.5),
          Map.entry(5.25, 5.5),
          Map.entry(5.75, 4.925));
}
