package frc.robot.controller;

import static edu.wpi.first.units.Units.Degrees;

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

  public static Operator init(Zones zone, CommandFactory commandFactory) {

    Operator operator = getInstance();

    operator.setZone(zone);
    operator.setCommandFactory(commandFactory);

    return operator;
  }

  @Override
  public Operator configureBindings() {
    // TODO: add methods to bind controller

    this.a()
        .onTrue(zone.setShootingOverideCommand(true))
        .onFalse(zone.setShootingOverideCommand(false));

    this.b().onTrue(commandFactory.goUnderTrenchCommand());

    this.x().onTrue(hood.setPosition(Degrees.of(20)));

    return this;
  }
}
