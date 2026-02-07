package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

public class FuelRollers {
  private PWMSparkMax sparkMaxController;

  public FuelRollers() {
    sparkMaxController = new PWMSparkMax(0);
  }

  public void startRollers() {
    sparkMaxController.set(-0.6);
  }

  public void stopRollers() {
    sparkMaxController.set(0);
  }
}
