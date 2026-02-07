package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;

public class TurretConstants {
  public static final Angle MIN_ANGLE = Units.Degrees.of(-135);
  public static final Angle MAX_ANGLE = Units.Degrees.of(135);

  // Meters
  public static final Transform2d TURRET_OFFSET = new Transform2d(0, 0, new Rotation2d());
}
