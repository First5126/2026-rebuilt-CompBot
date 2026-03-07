package frc.robot.constants;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;

public class FlyWheelConstants {
  public static double kP = 0.6;
  public static double kI = 0;
  public static double kD = 0.003;
  public static double kS = 0.32;
  public static double kV = 0.1168;
  public static double kA = 0;

  public static final int gearRatio = 2;
  public static final Distance radius = Meters.of(0.0508);
}
