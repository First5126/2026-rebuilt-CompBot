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

  
}
