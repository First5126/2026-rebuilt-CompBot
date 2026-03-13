package frc.robot.controller;

import java.util.Map;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import frc.robot.FMS.ShiftData;
import frc.robot.FMS.Zones;
import frc.robot.constants.ControllerConstants;
import frc.robot.constants.ControllerConstants.OperatorState;
import frc.robot.subsystems.CommandFactory;
import frc.robot.subsystems.FlyWheel;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Turret;
import lombok.Getter;
import lombok.Setter;

public class Operator extends CustomXboxController implements Controller {
  // Singleton instance
  private static Operator INSTANCE;

  @Getter @Setter private CommandFactory commandFactory;
  @Getter @Setter private Turret turret;
  @Getter @Setter private Zones zone;
  @Getter @Setter private OperatorState operatorState;
  @Getter @Setter private FlyWheel flyWheel;
  @Getter @Setter private Hood hood;

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

  public static Operator init() {
    return getInstance();
  }

  @Override
  public Operator configureBindings() {

    

    this.a().onTrue(
      new SelectCommand<OperatorState>(
        Map.of(
            OperatorState.NORMAL, Commands.none(),
            OperatorState.OVERIDE, Commands.none()
        ),
        () -> operatorState)
    );

    this.b().onTrue(
      new SelectCommand<OperatorState>(
        Map.of(
            OperatorState.NORMAL, Commands.none(),
            OperatorState.OVERIDE, Commands.none()
        ),
        () -> operatorState)
    ).onFalse(
      new SelectCommand<OperatorState>(
        Map.of(
            OperatorState.NORMAL, Commands.none(),
            OperatorState.OVERIDE, Commands.none()
        ),
        () -> operatorState)
    );

    this.x().onTrue(
      new SelectCommand<OperatorState>(
        Map.of(
            OperatorState.NORMAL, Commands.none(),
            OperatorState.OVERIDE, Commands.none()
        ),
        () -> operatorState)
    );

    this.y().onTrue(
      new SelectCommand<OperatorState>(
        Map.of(
            OperatorState.NORMAL, Commands.none(),
            OperatorState.OVERIDE, Commands.none()
        ),
        () -> operatorState)
    );


    this.povUp().whileTrue(
      new SelectCommand<OperatorState>(
        Map.of(
            OperatorState.NORMAL, Commands.none(),
            OperatorState.OVERIDE, hood.moveAngleUpCommand()
        ),
        () -> operatorState)
    );

    this.povDown().whileTrue(
      new SelectCommand<OperatorState>(
        Map.of(
            OperatorState.NORMAL, Commands.none(),
            OperatorState.OVERIDE, hood.moveAngleDownCommand()
        ),
        () -> operatorState)
    );

    this.povLeft().whileTrue(
      new SelectCommand<OperatorState>(
        Map.of(
            OperatorState.NORMAL, Commands.none(),
            OperatorState.OVERIDE, Commands.none()
        ),
        () -> operatorState)
    );

    this.povRight().whileTrue(
      new SelectCommand<OperatorState>(
        Map.of(
            OperatorState.NORMAL, Commands.none(),
            OperatorState.OVERIDE, Commands.none()
        ),
        () -> operatorState)
    );



    this.back().onTrue(
      new SelectCommand<OperatorState>(
        Map.of(
            OperatorState.NORMAL, Commands.runOnce(() -> this.setOperatorState(OperatorState.OVERIDE)),
            OperatorState.OVERIDE, Commands.runOnce(() -> this.setOperatorState(OperatorState.NORMAL))
        ),
        () -> operatorState)
    );

    this.start().onTrue(
      new SelectCommand<OperatorState>(
        Map.of(
            OperatorState.NORMAL, commandFactory.resetFMSTime(),
            OperatorState.OVERIDE, commandFactory.resetFMSTime() 
        ),
        () -> operatorState)
    );

    return this;
  }
}