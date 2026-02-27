package frc.robot.constants;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;

public class IndexerConstants {
    public static double indexerKP = 5;
    public static double spindexerKP = 5;

    public static AngularVelocity indexerSpeed = RotationsPerSecond.of(3);
    public static AngularVelocity spindexerSpeed = RotationsPerSecond.of(3);
}
