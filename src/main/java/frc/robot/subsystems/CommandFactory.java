package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.FMS.Zones;
import frc.robot.constants.ControllerConstants.OperatorState;
import frc.robot.controller.Operator;

public class CommandFactory {

  private Turret m_turret;
  private Zones m_zone;
  private ShootingMechanism m_shootingMechanism;
  private FlyWheel m_flyWheel;
  private Hood m_hood;
  private Indexer m_indexer;
  private IntakeDeployer m_intakeDeployer;
  private Intake m_intake;

  public CommandFactory(
      Turret turret,
      Zones zone,
      ShootingMechanism m_shootingMechanism,
      FlyWheel flyWheel,
      Hood hood,
      Indexer indexer,
      IntakeDeployer intakeDeployer,
      Intake intake) {
    this.m_turret = turret;
    this.m_zone = zone;
    this.m_shootingMechanism = m_shootingMechanism;
    this.m_flyWheel = flyWheel;
    this.m_hood = hood;
    this.m_indexer = indexer;
    this.m_intakeDeployer = intakeDeployer;
    this.m_intake = intake;
  }

  // Clear a potential jam by running the flywheel inward while reversing the indexer
  public Command clearShootingJam() {
    return m_flyWheel.shootInCommand().alongWith(m_indexer.reverseIndexing());
  }

  // Stop both the flywheel and the indexer
  public Command stopFlywheel() {
    return m_flyWheel.stopSpinning();
  }

  /* sets the hood position for going under the trench */
  public Command setHoodToTrenchPosition() {
    return m_hood.holdCertainPosition(Degrees.of(0));
  }

  public Command duckHood() {
    return m_hood
        .holdCertainPosition(Degrees.of(0))
        .withInterruptBehavior(InterruptionBehavior.kCancelSelf);
  }

  public Command stopEverything() {
    return m_flyWheel
        .stopSpinningOnce()
        .alongWith(m_indexer.stopIndexing())
        .alongWith(m_intake.stopIntake());
  }

  public Command intake() {
    return m_intake.runIntake();
  }

  public Command intakeAndShoot() {
    Command intake = m_intake.runIntake();
    Command turretMovement = m_shootingMechanism.startTrackingCommand();
    Command shoot =
        m_flyWheel
            .setSpeedWithSolution(m_shootingMechanism::getShootingSolution)
            .alongWith(m_hood.setPosition(m_shootingMechanism::getShootingSolution));

    return intake.alongWith(turretMovement).andThen(shoot);
  }

  // Start flywheel + hood positioning using the ShootingMechanism's computed solution when in
  // NORMAL state
  public Command startShootingWithSolution() {
    return m_flyWheel
        .setSpeedWithSolution(m_shootingMechanism::getShootingSolution)
        .alongWith(m_hood.setPosition(m_shootingMechanism::getShootingSolution));
  }

  public Command indexAndShoot() {
    Command startIndex = m_indexer.startIndexing();
    return startIndex;
  }

  public Command stopIndexAndShoot() {
    Command stopIndex = m_indexer.stopIndexing();
    return stopIndex;
  }

  private boolean isNormalOperatingState() {
    OperatorState state = Operator.getInstance().getOperatorState();
    return state == null || state == OperatorState.NORMAL;
  }

  public ConditionalCommand startTurretTracking() {
    return new ConditionalCommand(
        m_shootingMechanism.startTrackingCommand(), Commands.none(), this::isNormalOperatingState);
    /*
    Commands.defer(
        () -> {
          if (isNormalOperatingState()) {
            Command turretCommand = m_shootingMechanism.startTrackingCommandAuto();
            turretCommand.addRequirements(m_turret, m_shootingMechanism);
            return turretCommand;
          } else {
            Command none = Commands.none();
            none.addRequirements(m_shootingMechanism);
            return none;
          }
        },
        Set.of(m_shootingMechanism, m_turret));
        */
  }

  // Convenience / clearer naming for indexing control
  public Command startIndexer() {
    return m_indexer.startIndexing();
  }

  public Command stopIndexer() {
    return m_indexer.stopIndexing();
  }

  public Command test() {
    return Commands.none();
  }

  public Command setTurretToZero() {
    return m_turret.holdCertainPosition(Degrees.of(0));
  }

  public Command lowerIntake() {
    return m_intakeDeployer.lowerIntakeDownCommand();
  }

  public Command startIntake() {
    return m_intake.runIntake();
  }

  public Command reverseIntake() {
    return m_intake.runOuttake();
  }

  public Command stopIntake() {
    return m_intake.stopIntake();
  }

  public Command setIntakeDown() {
    return m_intakeDeployer.lowerIntakeDownCommand();
  }

  public Command incrementTurretOffset(double offsetDegrees) {
    return m_turret.adjustPositionDynamically(offsetDegrees);
  }
}