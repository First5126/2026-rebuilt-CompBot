package frc.robot.constants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RevolutionsPerSecond;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public class IntakeDeployerConstants {
  public static double INTAKE_KP = 80;

  public static int GEAR_RATIO = 25;

  public static Angle INTAKE_UP = Degrees.of(0);
  public static Angle INTAKE_HALFWAY_DOWN = Rotations.of(-0.1);
  public static Angle INTAKE_DOWN = Degrees.of(90);

  public static final AngularVelocity INTAKE_SPEED =
      RevolutionsPerSecond.of(20); // 1 revolution per second
  public static final AngularVelocity OUTTAKE_SPEED =
      RevolutionsPerSecond.of(-20); // -60 revolution per second

  public static Angle MAX_AGITATION_HEIGHT = Rotations.of(-0.08);

  public static int CRUISE_VELOCITY = 2; // Target cruise velocity of 80 rps
  public static int ACCELERATION = 8; // Target acceleration of 160 rps/s (0.5 seconds)
}
