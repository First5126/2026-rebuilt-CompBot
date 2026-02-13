package frc.robot.constants;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;

public class IntakeConstants {
    // TODO: find a good kP Value
    public static final double kP = 1;

    public static final AngularVelocity intakeSpeed = RotationsPerSecond.of(10);
}
