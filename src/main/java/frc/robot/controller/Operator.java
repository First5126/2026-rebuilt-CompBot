package frc.robot.controller;

import frc.robot.FMS.Zones;
import frc.robot.constants.ControllerConstants;
import frc.robot.subsystems.CommandFactory;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Turret;
import frc.robot.vision.AprilTagLocalization;
import lombok.Getter;
import lombok.Setter;

public class Operator extends CustomXboxController implements Controller {
  // Singleton instance
  private static Operator INSTANCE;

  @Getter @Setter private AprilTagLocalization aprilTagLocalization;
  @Getter @Setter private CommandFactory commandFactory;
  @Getter @Setter private Intake intake;
  @Getter @Setter private Turret turret;
  @Getter @Setter private Zones zone;
  @Getter @Setter private Hood hood;

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

  public static Operator init(Zones zone, CommandFactory commandFactory, Hood hood, Intake intake) {

    Operator operator = getInstance();

    operator.setZone(zone);
    operator.setCommandFactory(commandFactory);
    operator.setHood(hood);
    operator.setIntake(intake);

    return operator;
  }

  @Override
  public Operator configureBindings() {
    // TODO: add methods to bind controller

    this.leftBumper().onTrue(commandFactory.raiseIntake());
    this.rightBumper().onTrue(commandFactory.lowerIntake());

    this.rightTrigger().whileTrue(intake.runIntakeCommand()).onFalse(intake.stopIntakeCommand());
    this.leftTrigger().whileTrue(intake.runOuttakeCommand()).onFalse(intake.stopIntakeCommand());

    this.b()
        .whileTrue(commandFactory.startShootingMechanism())
        .onTrue(commandFactory.stopShootingMechanism());
    this.a().onTrue(commandFactory.startIndexing()).onFalse(commandFactory.stopIndexing());

    return this;
  }
}
