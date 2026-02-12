package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

public class FuelRollers {
  private PWMSparkMax sparkMaxController;

  /** Creates the fuel rollers subsystem. */
  public FuelRollers() {
    sparkMaxController = new PWMSparkMax(0);
  }

  /** Starts the rollers at intake speed. */
  public void startRollers() {
    sparkMaxController.set(-0.6);
  }

  /** Stops the rollers. */
  public void stopRollers() {
    sparkMaxController.set(0);
  }
}
