package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FuelRollers extends SubsystemBase {
  private PWMSparkMax sparkMaxController;

  /** Creates the fuel rollers subsystem. */
  public FuelRollers() {
    sparkMaxController = new PWMSparkMax(0);
  }

  public Command startRollersCommand() {
    return runOnce(() -> {
      startRollers();
    });
  }

  public Command stopRollersCommand() {
    return runOnce(() -> {
      stopRollers();
    });
  }

  /** Starts the rollers at intake speed. */
  private void startRollers() {
    sparkMaxController.set(-0.6);
  }

  /** Stops the rollers. */
  private void stopRollers() {
    sparkMaxController.set(0);
  }
}
