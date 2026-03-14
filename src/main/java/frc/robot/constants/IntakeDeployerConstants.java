package frc.robot.constants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RevolutionsPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public class IntakeDeployerConstants {
  public static double intakeKP = 0.2;

  public static Angle INTAKE_UP = Degrees.of(0);
  public static Angle INTAKE_DOWN = Degrees.of(90);

  public static final AngularVelocity INTAKE_SPEED =
      RevolutionsPerSecond.of(20); // 1 revolution per second
  public static final AngularVelocity OUTTAKE_SPEED =
      RevolutionsPerSecond.of(-20); // -60 revolution per second

  public static int CRUISE_VELOCITY = 2; // Target cruise velocity of 80 rps
  public static int ACCELERATION = 4; // Target acceleration of 160 rps/s (0.5 seconds)
}
