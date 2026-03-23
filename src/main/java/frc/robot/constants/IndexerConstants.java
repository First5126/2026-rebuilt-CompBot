package frc.robot.constants;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Voltage;

public class IndexerConstants {
  public static double indexerKP = 5;
  public static double spindexerKP = 5;

  public static final Voltage indexerSpeed = Volts.of(8);
  public static final Voltage spindexerSpeed = Volts.of(6);

  public static final Voltage indexerSpeedReverse = Volts.of(-4);
  public static final Voltage spindexerSpeedReverse = Volts.of(-3);
}
