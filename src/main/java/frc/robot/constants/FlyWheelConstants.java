package frc.robot.constants;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;

public class FlyWheelConstants {
  public static double kP = 100;

  public static AngularVelocity flyWheelSpeed = RotationsPerSecond.of(-100);
}
