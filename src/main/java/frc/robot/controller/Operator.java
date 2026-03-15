package frc.robot.controller;

import frc.robot.constants.ControllerConstants;
import frc.robot.subsystems.CommandFactory;
import lombok.Getter;
import lombok.Setter;

public class Operator extends CustomXboxController implements Controller {
  // Singleton instance
  private static Operator INSTANCE;

  @Getter @Setter private CommandFactory commandFactory;

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

  public static Operator init(CommandFactory commandFactory) {

    Operator operator = getInstance();

    operator.setCommandFactory(commandFactory);

    return operator;
  }

  @Override
  public Operator configureBindings() {
    // TODO: add methods to bind controller

    this.leftBumper().onTrue(commandFactory.raiseIntake());
    this.rightBumper().onTrue(commandFactory.lowerIntake());

    this.rightTrigger().onTrue(commandFactory.startIntake()).onFalse(commandFactory.stopIntake());
    this.leftTrigger().onTrue(commandFactory.reverseIntake()).onFalse(commandFactory.stopIntake());

    this.b()
        .whileTrue(commandFactory.startShootingMechanism())
        .onFalse(commandFactory.stopShootingMechanism());
    this.a().onTrue(commandFactory.startIndexing()).onFalse(commandFactory.stopIndexing());

    return this;
  }
}
