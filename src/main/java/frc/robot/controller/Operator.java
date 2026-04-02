package frc.robot.controller;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FMS.Zones;
import frc.robot.RobotLogger;
import frc.robot.constants.ControllerConstants;
import frc.robot.constants.ControllerConstants.OperatorState;
import frc.robot.subsystems.CommandFactory;
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
  public static RobotLogger logger = new RobotLogger("Operator");
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

    this.a().whileTrue(commandFactory.setHoodToTrenchPosition());

    this.y().whileTrue(commandFactory.stopFlywheel());

    this.b()
        .onTrue(commandFactory.startIndexer())
        .onFalse(
            commandFactory
                .clearShootingJam()
                .andThen(Commands.waitSeconds(0.25))
                .andThen(commandFactory.stopIndexer()));

    this.x()
        .onTrue(commandFactory.clearShootingJam())
        .onFalse(commandFactory.stopFlywheelAndIndexer());

    this.leftBumper()
        .whileTrue(commandFactory.agitateIntake().alongWith(commandFactory.startIntake()));

    this.rightBumper().onTrue(commandFactory.lowerIntake());

    this.rightTrigger().onTrue(commandFactory.startIntake()).onFalse(commandFactory.stopIntake());

    this.leftTrigger().onTrue(commandFactory.reverseIntake()).onFalse(commandFactory.stopIntake());

    /*this.start().onTrue(Commands.runOnce(() -> ShiftData.resetMatchTimeCalibration()));*/

    this.back().onTrue(this.changeOperatorStateCommand());
    this.povLeft().onTrue(commandFactory.incrementTurretOffset(1));
    this.povRight().onTrue(commandFactory.incrementTurretOffset(-1));

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
            logger.logAndDisplay("Operator OVERRIDE Active", true);

          } else {
            this.setOperatorState(OperatorState.NORMAL);
            logger.logAndDisplay("Operator OVERRIDE Active", false);
          }
        });
  }
}
