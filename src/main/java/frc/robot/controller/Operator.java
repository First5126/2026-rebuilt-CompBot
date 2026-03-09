package frc.robot.controller;

import frc.robot.constants.ControllerConstants;
import frc.robot.subsystems.IntakeDeployer;
import lombok.Getter;
import lombok.Setter;

public class Operator extends CustomXboxController implements Controller {
  // Singleton instance
  private static Operator INSTANCE;

  @Getter @Setter private static IntakeDeployer intakeDeployer;

  // Private constructor to prevent instantiation from outside
  private Operator() {
    super(ControllerConstants.OPERATOR_CONTROLLER_PORT);
  }

  // Public method to access the single instance
  public static Operator getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new Operator();
    }
    return INSTANCE;
  }

  public static Operator init(
    IntakeDeployer intakeDeployer
  ) {
    Operator.setIntakeDeployer(intakeDeployer);
    return getInstance();
  }

  @Override
  public Operator configureBindings() {
    // TODO: add methods to bind controller
    this.a().whileTrue(intakeDeployer.runIntakeWheelsCommand());
    this.b().whileTrue(intakeDeployer.runOuttakeWheelsCommand());

    return this;
  }
}
