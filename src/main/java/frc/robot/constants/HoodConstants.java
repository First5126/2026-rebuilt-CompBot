package frc.robot.constants;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Angle;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import java.util.Map;

public class HoodConstants {
  public static double kP = 135.0;
  public static double kI = 0.0;
  public static double kD = 0.0;
  public static double kS = 0.35;
  public static double kV = 13.84;
  
  public static final Angle minimumHoodAngle = Degrees.of(42);
  public static final Angle maximumHoodAngle = Degrees.of(69);
  public static final Angle zeroOffset = Degrees.of(21);

  public static final Angle lowerLimit = Rotations.of(0);
  public static final Angle upperLimit = Rotations.of(0.075);

  // TODO: Very important to fix this so we dont rip the hood off
  public static final InterpolatingDoubleTreeMap DISTANCE_TO_ANGLE_INTERPOLATOR =
      InterpolatingDoubleTreeMap.ofEntries(
          Map.entry(0.0, 0.0), Map.entry(2.0, 0.5), Map.entry(5.0, 1.0));
}
