package frc.robot.constants;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;

public class IntakeDeployerConstants {
  public static double intakeKP = 40;

  public static Angle INTAKE_UP = Degrees.of(0);
  public static Angle INTAKE_DOWN = Degrees.of(72);

  public static int CRUISE_VELOCITY = 2; // Target cruise velocity of 80 rps
  public static int ACCELERATION = 4; // Target acceleration of 160 rps/s (0.5 seconds)
}
