package frc.robot.controller;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.FMS.ShiftData;
import frc.robot.FMS.Zones;
import frc.robot.constants.ControllerConstants;
import frc.robot.constants.ControllerConstants.OperatorState;
import frc.robot.subsystems.CommandFactory;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.ShootingMechanism;
import java.util.Map;
import lombok.Getter;
import lombok.Setter;

public class Operator extends CustomXboxController implements Controller {
  // Singleton instance
  private static Operator INSTANCE;

  @Getter @Setter public OperatorState operatorState;

  @Getter @Setter private CommandFactory commandFactory;
  @Getter @Setter private Zones zone;
  @Getter @Setter private ShootingMechanism shootingMechanism;

  private Operator() {
    super(ControllerConstants.OPERATOR_CONTROLLER_PORT);
  }

  // Private constructor to prevent instantiation from outside
  public static Operator init(
      CommandFactory commandFactory,
      Zones zone,
      ShootingMechanism shootingMechanism,
      OperatorState operatorState) {
    Operator operator = getInstance();
    operator.setCommandFactory(commandFactory);
    operator.setZone(zone);
    operator.setOperatorState(OperatorState.NORMAL);
    operator.setShootingMechanism(shootingMechanism);

    return operator;
  }

  // Public method to access the single instance
  public static Operator getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new Operator();
    }
    return INSTANCE;
  }

  @Override
  public Operator configureBindings() {

    // 0.10 deadband
    Trigger rightJoystickX = new Trigger(() -> this.getRightX() > 0.10);
    Trigger rightJoystickY = new Trigger(() -> this.getRightY() > 0.10);

    //0.10 deadband
    Trigger leftJoystickX = new Trigger(() -> this.getLeftX() > 0.10);
    Trigger leftJoystickY = new Trigger(() -> this.getLeftY() > 0.10);

    this.a()
        .onTrue(
            new SelectCommand<OperatorState>(
                Map.of(
                    OperatorState.NORMAL, commandFactory.duckHood(),
                    OperatorState.OVERRIDE, commandFactory.startIndexing()),
                () -> operatorState))
        .onFalse(
            new SelectCommand<OperatorState>(
                Map.of(
                    OperatorState.NORMAL, Commands.none(),
                    OperatorState.OVERRIDE, commandFactory.stopIndexing()),
                () -> operatorState));

    this.b()
        .onTrue(
            new SelectCommand<OperatorState>(
                Map.of(
                    OperatorState.NORMAL, commandFactory.shootCommand(),
                    OperatorState.OVERRIDE,
                        commandFactory.startFlywheelsWithSolution(
                            shootingMechanism::getShootingSolution)),
                () -> operatorState))
        .onFalse(
            new SelectCommand<OperatorState>(
                Map.of(
                    OperatorState.NORMAL, commandFactory.stopShootCommand(),
                    OperatorState.OVERRIDE, commandFactory.stopShooting()),
                () -> operatorState));

    this.x()
        .onTrue(
            new SelectCommand<OperatorState>(
                Map.of(
                    OperatorState.NORMAL, Commands.none(),
                    OperatorState.OVERRIDE, commandFactory.reverseShootingCommand()),
                () -> operatorState))
        .onFalse(
            new SelectCommand<OperatorState>(
                Map.of(
                    OperatorState.NORMAL, Commands.none(),
                    OperatorState.OVERRIDE, commandFactory.stopShootingCommand()),
                () -> operatorState));

    this.y()
        .onTrue(
            new SelectCommand<OperatorState>(
                Map.of(
                    OperatorState.NORMAL, Commands.none(),
                    OperatorState.OVERRIDE, Commands.none()),
                () -> operatorState));

    this.povUp()
        .whileTrue(
            new SelectCommand<OperatorState>(
                Map.of(
                    OperatorState.NORMAL, Commands.none(),
                    OperatorState.OVERRIDE, commandFactory.slowlyMoveHoodUp()),
                () -> operatorState));

    this.povDown()
        .whileTrue(
            new SelectCommand<OperatorState>(
                Map.of(
                    OperatorState.NORMAL, Commands.none(),
                    OperatorState.OVERRIDE, commandFactory.slowlyMoveHoodUp()),
                () -> operatorState));

    this.povLeft()
        .whileTrue(
            new SelectCommand<OperatorState>(
                Map.of(
                    OperatorState.NORMAL, Commands.none(),
                    OperatorState.OVERRIDE, commandFactory.moveTurretManualy(Degrees.of(0.1))),
                () -> operatorState));

    this.povRight()
        .whileTrue(
            new SelectCommand<OperatorState>(
                Map.of(
                    OperatorState.NORMAL, Commands.none(),
                    OperatorState.OVERRIDE, commandFactory.moveTurretManualy(Degrees.of(0.1))),
                () -> operatorState));

    leftJoystickX.whileTrue(commandFactory.moveTurretManualyWithSticks(this::getLeftX));
    rightJoystickY.whileTrue(commandFactory.moveHoodManualyWithSticks(this::getRightY));

    this.start().onTrue(Commands.runOnce(() -> ShiftData.resetMatchTimeCalibration()));

    this.back().onTrue(changeOperatorStateCommand());

    return this;
  }

  private Command changeOperatorStateCommand() {
    return Commands.runOnce(
        () -> {
          if (operatorState == OperatorState.NORMAL) {
            this.setOperatorState(OperatorState.OVERRIDE);
            SmartDashboard.putBoolean("Operator OVERRIDE Active", true);

          } else {
            this.setOperatorState(OperatorState.NORMAL);
            SmartDashboard.putBoolean("Operator OVERRIDE Active", false);
          }
        });
  }
}
