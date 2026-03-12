package frc.robot.constants;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import java.util.Map;

public class TurretConstants {
  public static final Angle MIN_ANGLE = Units.Degrees.of(-162);
  public static final Angle MAX_ANGLE = Units.Degrees.of(162);

  public static final double kP = 90;
  public static final double kI = 0;
  public static final double kD = 0;
  public static final double kS = 0.32;
  public static final double kV = 0.106;
  public static final double kA = 0;

  public static final double CURRENT_LIMIT = 30;
  public static final double STATOR_LIMIT = 30;

  public static final Angle ENCODER_OFFSET = Units.Rotations.of(-0.433);

  // Meters per second at 12 volts. This is used for feedforward calculations, and should be
  // measured at the output of the turret gearbox.
  public static final Transform2d TURRET_OFFSET = new Transform2d(0, 0, new Rotation2d());
  public static final int gearRatio = 100;

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
