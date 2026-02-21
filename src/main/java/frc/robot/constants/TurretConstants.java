package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import java.util.Map;

public class TurretConstants {
  public static final Angle MIN_ANGLE = Units.Degrees.of(-135);
  public static final Angle MAX_ANGLE = Units.Degrees.of(135);

  public static final double kP = 96;
  public static final double kI = 0;
  public static final double kD = 0;

  // Meters
  public static final Transform2d TURRET_OFFSET = new Transform2d(0, 0, new Rotation2d());

  // Distance first then Time
  public static final InterpolatingDoubleTreeMap DISTANCE_TO_TIME_INTERPOLATOR =
      InterpolatingDoubleTreeMap.ofEntries(
          Map.entry(0.0, 0.0), Map.entry(2.0, 0.5), Map.entry(5.0, 1.0));
}
