package frc.robot.controller;

import static edu.wpi.first.units.Units.Degree;

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

  // Private constructor to prevent instantiation from outside
  public static Operator init(
      CommandFactory commandFactory, Turret turret, Zones zone, FlyWheel flyWheel, Hood hood) {
    Operator operator = getInstance();
    operator.setCommandFactory(commandFactory);
    operator.setTurret(turret);
    operator.setZone(zone);
    operator.setFlyWheel(flyWheel);
    operator.setHood(hood);
    operator.setOperatorState(OperatorState.NORMAL);

    return operator;
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
    this.povRight().onTrue(commandFactory.manualTurretRotation(Degree.of(45)));
    this.povLeft().onTrue(commandFactory.manualTurretRotation(Degree.of(-45)));
    this.povUp().whileTrue(commandFactory.manualHoodRotation(Degree.of(0.5)));
    this.povDown().whileTrue(commandFactory.manualHoodRotation(Degree.of(-0.5)));

    this.b().whileTrue(commandFactory.startShootingMechanism());
    this.b().onFalse(commandFactory.stopShootingMechanism());

    this.a().onTrue(commandFactory.startIndexing());
    this.a().onFalse(commandFactory.stopIndexing());
    return this;
  }
}