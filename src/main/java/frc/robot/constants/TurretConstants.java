package frc.robot.constants;

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
  public static final Transform2d TURRET_OFFSET = new Transform2d(-0.127, 0, new Rotation2d());
  public static final int gearRatio = 100;
}
