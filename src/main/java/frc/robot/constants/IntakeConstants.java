package frc.robot.constants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RevolutionsPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public class IntakeConstants {
    public static final AngularVelocity INTAKE_SPEED =
      RevolutionsPerSecond.of(20); // 1 revolution per second
    public static final AngularVelocity OUTTAKE_SPEED =
      RevolutionsPerSecond.of(-20); // -60 revolution per second
    

    public static double kP = 10;
    public static double kI = 0;
    public static double kD = 0;
    public static double kS = 0;
    public static double kV = 0;
    public static double kA = 0;
}
