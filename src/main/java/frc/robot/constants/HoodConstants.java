package frc.robot.constants;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;

public class HoodConstants {
  public static double kP = 135.0;
  public static double kI = 0.0;
  public static double kD = 0.0;
  public static double kS = 0.35;
  public static double kV = 13.84;

  // Time it takes for the hood to reach 0
  public static double DUCK_SPEED = 0.25;

  // Hood hard/soft travel limits (mechanism angle as configured for the TalonFX fused sensor).
  public static final Angle MIN_ANGLE = Degrees.of(0);
  public static final Angle MAX_ANGLE = Degrees.of(6);

  public static final double MAX_VOLTAGE_MANUAL = 1;

  public static final double MAGNETIC_OFFSET = 0.217529296875;

  // TODO: Very important to fix this so we dont rip the hood off

}
