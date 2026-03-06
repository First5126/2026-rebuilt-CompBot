package frc.robot.constants;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;

public class FlyWheelConstants {
  public static double kP = 100;

  public static AngularVelocity flyWheelSpeed = RotationsPerSecond.of(-100);
  
  public final static int gearRatio = 2;
  public final static Distance radius = Meters.of(0.0508);
}
