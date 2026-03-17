package frc.robot.controller;

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
import java.util.Map;
import lombok.Getter;
import lombok.Setter;

/**
 * Operator controller wrapper.
 *
 * <p>This class is a singleton that extends CustomXboxController and is responsible for: - Holding
 * the current OperatorState (NORMAL or OVERRIDE) - Holding a reference to the CommandFactory used
 * to create commands bound to buttons - Configuring button-to-command bindings for the operator
 * controller
 *
 * <p>Bindings frequently use SelectCommand to choose behavior based on the current OperatorState.
 */
public class Operator extends CustomXboxController implements Controller {
  // Singleton instance
  private static Operator INSTANCE;

  /**
   * The current operator state.
   *
   * <p>Used by SelectCommand mappings to determine which command should run for a given button.
   */
  @Getter @Setter public OperatorState operatorState;

  /**
   * Factory for creating commands bound to this controller.
   *
   * <p>Must be set (via init) before configureBindings is called.
   */
  @Getter @Setter private CommandFactory commandFactory;

  /** The current FMS zone. Present for potential FMS-dependent command selection. */
  @Getter @Setter private Zones zone;

  /**
   * Private constructor to enforce singleton usage.
   *
   * <p>Initializes the underlying CustomXboxController on the configured operator port.
   */
  private Operator() {
    super(ControllerConstants.OPERATOR_CONTROLLER_PORT);
  }

  /**
   * Initialize or update the singleton Operator with a CommandFactory and initial state.
   *
   * <p>If the singleton does not yet exist it will be created. This method sets the commandFactory
   * on the singleton and returns the instance.
   *
   * @param commandFactory factory used to create operator commands
   * @param operatorState initial operator state (may be ignored by callers that set state later)
   * @return the singleton Operator instance
   */
  public static Operator init(CommandFactory commandFactory, OperatorState operatorState) {
    Operator operator = getInstance();
    operator.setCommandFactory(commandFactory);
    operator.setOperatorState(operatorState);

    return operator;
  }

  /**
   * Returns the singleton instance of Operator, creating it if necessary.
   *
   * @return the Operator singleton
   */
  public static Operator getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new Operator();
    }
    return INSTANCE;
  }

  /**
   * Alternate initializer that only requires a CommandFactory.
   *
   * <p>Convenience overload used by code that manages OperatorState separately.
   *
   * @param commandFactory factory used to create operator commands
   * @return the singleton Operator instance
   */
  public static Operator init(CommandFactory commandFactory) {

    Operator operator = getInstance();

    operator.setCommandFactory(commandFactory);

    return operator;
  }

  /**
   * Configure button bindings for the operator controller.
   *
   * <p>This method: - Creates triggers for joystick deadbands. - Binds controller buttons to
   * SelectCommand instances that pick behavior by operatorState. - Attaches manual turret/hood
   * control to joystick axes. - Sets up start/back button special behaviors (match time calibration
   * reset and state toggle).
   *
   * <p>Returns the Operator instance to allow fluent usage.
   *
   * @return this Operator (configured)
   */
  @Override
  public Operator configureBindings() {

    // 0.10 deadband
    Trigger rightJoystickX = new Trigger(() -> this.getRightX() > 0.10);
    Trigger rightJoystickY = new Trigger(() -> this.getRightY() > 0.10);

    // 0.10 deadband
    Trigger leftJoystickX = new Trigger(() -> this.getLeftX() > 0.10);
    Trigger leftJoystickY = new Trigger(() -> this.getLeftY() > 0.10);

    this.a()
        .onTrue(
            new SelectCommand<OperatorState>(
                Map.of(
                    OperatorState.NORMAL, commandFactory.lowerIntake(),
                    OperatorState.OVERRIDE, commandFactory.startIndexing()),
                () -> operatorState))
        .onFalse(
            new SelectCommand<OperatorState>(
                Map.of(
                    OperatorState.NORMAL, commandFactory.raiseIntake(),
                    OperatorState.OVERRIDE, commandFactory.stopIndexing()),
                () -> operatorState))
        .whileTrue(
            new SelectCommand<OperatorState>(
                Map.of(
                    OperatorState.NORMAL, commandFactory.duckHood(),
                    OperatorState.OVERRIDE, Commands.none()),
                () -> operatorState));

    this.b()
        .onTrue(
            new SelectCommand<OperatorState>(
                Map.of(
                    OperatorState.NORMAL, commandFactory.shootCommand(),
                    OperatorState.OVERRIDE, commandFactory.startFlywheelsWithSolution()),
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

    /*
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
                () -> operatorState));\
    */

    this.leftBumper()
        .onTrue(
            new SelectCommand<OperatorState>(
                Map.of(
                    OperatorState.NORMAL, commandFactory.raiseIntake(),
                    OperatorState.OVERRIDE, commandFactory.raiseIntake()),
                () -> operatorState));

    this.rightBumper()
        .onTrue(
            new SelectCommand<OperatorState>(
                Map.of(
                    OperatorState.NORMAL, commandFactory.lowerIntake(),
                    OperatorState.OVERRIDE, commandFactory.lowerIntake()),
                () -> operatorState));

    this.rightTrigger()
        .onTrue(
            new SelectCommand<OperatorState>(
                Map.of(
                    OperatorState.NORMAL, commandFactory.startIntake(),
                    OperatorState.OVERRIDE, commandFactory.reverseIntake()),
                () -> operatorState))
        .onFalse(
            new SelectCommand<OperatorState>(
                Map.of(
                    OperatorState.NORMAL, commandFactory.stopIntake(),
                    OperatorState.OVERRIDE, commandFactory.stopIntake()),
                () -> operatorState));

    this.leftTrigger()
        .onTrue(
            new SelectCommand<OperatorState>(
                Map.of(
                    OperatorState.NORMAL, Commands.none(),
                    OperatorState.OVERRIDE, commandFactory.startIntake()),
                () -> operatorState))
        .onFalse(
            new SelectCommand<OperatorState>(
                Map.of(
                    OperatorState.NORMAL, Commands.none(),
                    OperatorState.OVERRIDE, commandFactory.stopIntake()),
                () -> operatorState));

    // leftJoystickX.whileTrue(commandFactory.moveTurretManualyWithSticks(this::getLeftX));
    // rightJoystickY.whileTrue(commandFactory.moveHoodManualyWithSticks(this::getRightY));

    this.start().onTrue(Commands.runOnce(() -> ShiftData.resetMatchTimeCalibration()));

    this.back().onTrue(changeOperatorStateCommand());

    return this;
  }

  /**
   * Create a command that toggles between NORMAL and OVERRIDE operator states.
   *
   * <p>The returned command runs once and also updates SmartDashboard with the new OVERRIDE
   * indicator boolean ("Operator OVERRIDE Active").
   *
   * @return a one-shot Command that toggles the operatorState
   */
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
