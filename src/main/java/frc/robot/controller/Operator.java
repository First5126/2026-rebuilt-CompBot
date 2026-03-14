package frc.robot.controller;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import frc.robot.FMS.ShiftData;
import frc.robot.FMS.Zones;
import frc.robot.constants.ControllerConstants;
import frc.robot.constants.ControllerConstants.OperatorState;
import frc.robot.subsystems.CommandFactory;
import frc.robot.subsystems.FlyWheel;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.ShootingMechanism;
import frc.robot.subsystems.Turret;
import java.util.Map;
import lombok.Getter;
import lombok.Setter;

public class Operator extends CustomXboxController implements Controller {
  // Singleton instance
  private static Operator INSTANCE;

  @Getter @Setter public OperatorState operatorState;

  @Getter @Setter private CommandFactory commandFactory;
  @Getter @Setter private Turret turret;
  @Getter @Setter private Zones zone;
  @Getter @Setter private FlyWheel flyWheel;
  @Getter @Setter private Hood hood;
  @Getter @Setter private ShootingMechanism shootingMechanism;
  @Getter @Setter private Command turretDefaultCommand;
  @Getter @Setter private Command hoodDefaultCommand;

  private Operator() {
    super(ControllerConstants.OPERATOR_CONTROLLER_PORT);
  }

  // Private constructor to prevent instantiation from outside
  public static Operator init(
      CommandFactory commandFactory,
      Turret turret,
      Zones zone,
      FlyWheel flyWheel,
      Hood hood,
      ShootingMechanism shootingMechanism,
      OperatorState operatorState) {
    Operator operator = getInstance();
    operator.setCommandFactory(commandFactory);
    operator.setTurret(turret);
    operator.setZone(zone);
    operator.setFlyWheel(flyWheel);
    operator.setHood(hood);
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

  public static Operator init(CommandFactory commandFactory) {
    Operator operator = getInstance();
    operator.setCommandFactory(commandFactory);
    return operator;
  }

  @Override
  public Operator configureBindings() {

    this.a()
        .onTrue(
            new SelectCommand<OperatorState>(
                Map.of(
                    OperatorState.NORMAL, Commands.none(),
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
                        flyWheel.setSpeedWithSolution(shootingMechanism::getShootingSolution)),
                () -> operatorState))
        .onFalse(
            new SelectCommand<OperatorState>(
                Map.of(
                    OperatorState.NORMAL, commandFactory.stopShootCommand(),
                    OperatorState.OVERRIDE, flyWheel.stopSpinning()),
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
                    OperatorState.OVERRIDE, hood.moveAngleUpCommand()),
                () -> operatorState));

    this.povDown()
        .whileTrue(
            new SelectCommand<OperatorState>(
                Map.of(
                    OperatorState.NORMAL, Commands.none(),
                    OperatorState.OVERRIDE, hood.moveAngleDownCommand()),
                () -> operatorState));

    this.povLeft()
        .whileTrue(
            new SelectCommand<OperatorState>(
                Map.of(
                    OperatorState.NORMAL, Commands.none(),
                    OperatorState.OVERRIDE, turret.manualRotation(Degrees.of(0.1))),
                () -> operatorState));

    this.povRight()
        .whileTrue(
            new SelectCommand<OperatorState>(
                Map.of(
                    OperatorState.NORMAL, Commands.none(),
                    OperatorState.OVERRIDE, turret.manualRotation(Degrees.of(-0.1))),
                () -> operatorState));

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

            this.setHoodDefaultCommand(hood.getDefaultCommand());
            this.setTurretDefaultCommand(turret.getDefaultCommand());

            hood.setDefaultCommand(Commands.run(() -> {}, hood));
            turret.setDefaultCommand(Commands.run(() -> {}, turret));

          } else {
            this.setOperatorState(OperatorState.NORMAL);
            SmartDashboard.putBoolean("Operator OVERRIDE Active", false);

            hood.setDefaultCommand(hoodDefaultCommand);
            turret.setDefaultCommand(turretDefaultCommand);
          }
        });
  }
}
