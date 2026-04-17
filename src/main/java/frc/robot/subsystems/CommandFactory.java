package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FMS.Zones;
import frc.robot.constants.ControllerConstants.OperatorState;
import frc.robot.constants.WaypointConstants;
import frc.robot.controller.Operator;
import java.util.Set;
import java.util.function.Supplier;

/**
 * Factory of high-level Command objects that coordinate multiple subsystems.
 *
 * <p>CommandFactory provides named and reusable commands (composite and single-shot) that the
 * RobotContainer and operator bindings can use.
 */
public class CommandFactory {

  private CommandSwerveDrivetrain m_drivetrain;
  private int m_side = 1;
  private Turret m_turret;
  private Zones m_zone;
  private ShootingMechanism m_shootingMechanism;
  private FlyWheel m_flyWheel;
  private Hood m_hood;
  private Indexer m_indexer;
  private IntakeDeployer m_intakeDeployer;
  private Intake m_intake;

  public CommandFactory(
      CommandSwerveDrivetrain drivetrain,
      Turret turret,
      Zones zone,
      ShootingMechanism m_shootingMechanism,
      FlyWheel flyWheel,
      Hood hood,
      Indexer indexer,
      IntakeDeployer intakeDeployer,
      Intake intake) {
    this.m_drivetrain = drivetrain;
    this.m_turret = turret;
    this.m_zone = zone;
    this.m_zone = zone;
    this.m_shootingMechanism = m_shootingMechanism;
    this.m_flyWheel = flyWheel;
    this.m_hood = hood;
    this.m_indexer = indexer;
    this.m_intakeDeployer = intakeDeployer;
    this.m_intake = intake;
  }

  public Command driveCircle() {
    return Commands.defer(
            () -> {
              switch (m_side) {
                case 1:
                  System.out.println("Heading To BottomLeftCorner");
                  return m_drivetrain
                      .goToPose(WaypointConstants.BottomLeftCornner)
                      .andThen(() -> m_side = 2);

                case 2:
                  System.out.println("Heading To TopLeftCorner");
                  return m_drivetrain
                      .goToPose(WaypointConstants.TopLeftCornner)
                      .andThen(() -> m_side = 3);

                case 3:
                  System.out.println("Heading To TopRightCorner");
                  return m_drivetrain
                      .goToPose(WaypointConstants.TopRightCornner)
                      .andThen(() -> m_side = 4);

                case 4:
                default:
                  System.out.println("Heading To BottomRightCorner");
                  return m_drivetrain
                      .goToPose(WaypointConstants.BottomRightCornner)
                      .andThen(() -> m_side = 1);
              }
            },
            Set.of(m_drivetrain))
        .repeatedly();
  }

  /**
   * Drives the robot around a sequence of corner waypoints in a repeating pattern.
   *
   * @return A repeating command that cycles between the defined corner waypoints
   */

  /**
   * Drives the robot around four corner waypoints in a repeating cycle (demo/test use).
   *
   * @return A repeating autonomous Command
   */

  /*public Command resetFMSTime() {
    return Commands.runOnce(
        () -> {
          ShiftData.resetMatchTimeCalibration();
        });
  }*/

  // Clear a potential jam by running the flywheel inward while reversing the indexer
  public Command clearShootingJam() {
    return m_flyWheel.shootInCommand().alongWith(m_indexer.reverseIndexing());
  }

  /**
   * Clears a potential shooting jam by running the flywheel inward while reversing the indexer.
   *
   * @return Command that executes the clear-jam routine
   */

  /**
   * Clears a shooting jam by running the flywheel inward and reversing the indexer.
   *
   * @return Command that executes the clear-jam routine
   */

  // Stop both the flywheel and the indexer
  public Command stopFlywheelAndIndexer() {
    return m_flyWheel.stopSpinning().alongWith(m_indexer.stopIndexing());
  }

  /**
   * Stops the flywheel and indexer subsystems together.
   *
   * @return Command that stops both the flywheel and the indexer
   */

  /**
   * Stops shooter and indexer together.
   *
   * @return Command that stops both subsystems
   */

  /* sets the hood position for going under the trench */
  public Command setHoodToTrenchPosition() {
    return m_hood.holdCertainPosition(Degrees.of(0));
  }

  /**
   * Sets the hood to the predefined "trench" position used for intake or low shots.
   *
   * @return Command that moves the hood to the trench angle
   */

  /**
   * Sets the hood to the predefined trench angle.
   *
   * @return Command that puts the hood to trench position
   */
  public Command rotateTurretBy(Angle amountOfMovement) {
    return m_turret.manualRotation(amountOfMovement);
  }

  /**
   * Rotates the turret by a relative amount.
   *
   * @param amountOfMovement relative angle to move turret
   * @return Command that performs the rotation
   */
  public Command stopEverything() {
    return Commands.runOnce(
        () -> {
          stopFlywheel();
          stopIndexer();
          stopIntake();
        });
  }

  /**
   * Convenience command that immediately stops main mechanisms (flywheel, indexer, intake).
   *
   * @return One-shot command that halts primary mechanisms
   */

  /**
   * Stops main mechanisms: flywheel, indexer and intake.
   *
   * @return One-shot command that stops subsystems
   */
  public Command rotateHoodBy(Angle amountOfMovement) {
    return m_hood.manualRotation(amountOfMovement);
  }

  /**
   * Rotates the hood by a relative angular amount.
   *
   * @param amountOfMovement amount of angle to rotate the hood by
   * @return Command performing the relative hood rotation
   */
  public Command spinFlywheelContinuously() {
    return m_flyWheel.rotateFlywheel();
  }

  /**
   * Spins the flywheel continuously (open-loop) until another command stops it.
   *
   * @return Command that spins the flywheel
   */

  /**
   * Spins up the flywheel (one-shot).
   *
   * @return Command that starts the flywheel
   */
  public Command stopFlywheel() {
    return m_flyWheel.stopSpinning();
  }

  /**
   * Stops the flywheel motors.
   *
   * @return Command that stops the flywheel
   */
  public Command duckHood() {
    return m_hood.holdCertainPosition(Degrees.of(0));
  }

  /**
   * Positions the hood in a stowed or "duck" posture (convenience alias).
   *
   * @return Command that moves the hood to the duck/stowed angle
   */
  public Command intake() {
    return m_intake.runIntake();
  }

  /**
   * Runs the intake rollers to acquire game pieces.
   *
   * @return Command that runs the intake
   */

  /**
   * Starts the intake rollers.
   *
   * @return Command that runs the intake
   */
  public Command intakeAndShoot() {
    Command intake = m_intake.runIntake();
    Command turretMovement = m_shootingMechanism.startTrackingCommand();
    Command shoot =
        m_flyWheel
            .setSpeedWithSolution(m_shootingMechanism::getShootingSolution)
            .alongWith(m_hood.setPosition(m_shootingMechanism::getShootingSolution));

    return intake.alongWith(turretMovement).andThen(shoot);
  }

  /**
   * Composite: run intake, begin turret tracking, then run the shooting routine once.
   *
   * @return Command sequence that intakes then shoots
   */

  /**
   * High-level sequence: intake, track target, then run shooting routine once.
   *
   * @return Composite command that intakes then shoots
   */

  // Start flywheel + hood positioning using the ShootingMechanism's computed solution when in
  // NORMAL state
  public Command startShootingWithSolution() {
    return m_flyWheel
        .setSpeedWithSolution(m_shootingMechanism::getShootingSolution)
        .alongWith(m_hood.setPosition(m_shootingMechanism::getShootingSolution));
  }

  /**
   * Starts the flywheel and positions the hood using the current computed shooting solution.
   *
   * @return Command that runs flywheel + hood positioning based on the shooting solution
   */
  public Command indexAndShoot() {
    // Command startFlyWheel = m_flyWheel
    //     .setSpeedWithSolution(m_shootingMechanism::getShootingSolution)
    //     .alongWith(m_hood.setPosition(m_shootingMechanism::getShootingSolution));
    Command startIndex = m_indexer.startIndexing();
    return startIndex;
  }

  /**
   * Starts indexing to feed game pieces into the shooter.
   *
   * @return Command that begins indexing
   */
  public Command stopIndexAndShoot() {
    Command stopIndex = m_indexer.stopIndexing();
    return stopIndex;
  }

  /**
   * Stops the indexer.
   *
   * @return Command that stops indexing
   */

  // Stop flywheel and stow hood to 0 degrees
  public Command stopFlywheelAndStowHood() {
    return m_flyWheel.stopSpinning().alongWith(m_hood.setPosition(Degree.of(0)));
  }

  /**
   * Stops the flywheel and stows the hood to zero degrees.
   *
   * @return Command that stops the flywheel and sets hood to zero
   */
  public Command startIndexing() {
    return m_indexer.startIndexing();
  }

  /**
   * Starts the indexer to feed balls into the shooter.
   *
   * @return Command that starts indexing
   */
  public Command stopIndexing() {
    return m_indexer.stopIndexing();
  }

  /**
   * Stops the indexer (convenience wrapper).
   *
   * @return Command that stops the indexer
   */
  private boolean isNormalOperatingState() {
    OperatorState state = Operator.getInstance().getOperatorState();
    return state == null || state == OperatorState.NORMAL;
  }

  public Command startTurretTracking() {
    return Commands.defer(
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
  }

  /**
   * Starts turret tracking if the operator state permits; otherwise returns a no-op command.
   *
   * @return Command that performs turret tracking or a no-op when disabled
   */

  // Convenience / clearer naming for indexing control
  public Command startIndexer() {
    return startIndexing();
  }

  /**
   * Alias for {@link #startIndexing()}.
   *
   * @return Command that starts the indexer
   */
  public Command rotateTurretToZero() {
    return m_turret.holdCertainPosition(Degrees.of(0));
  }

  /**
   * Commands the turret to hold its zero position.
   *
   * @return Command that holds turret at zero
   */
  public Command stopIndexer() {
    return stopIndexing();
  }

  /**
   * Alias for {@link #stopIndexing()}.
   *
   * @return Command that stops the indexer
   */
  public Command test() {
    return Commands.none();
  }

  /**
   * A placeholder/no-op test command.
   *
   * @return A no-op command
   */
  public Command startFlywheelWithSolution() {
    return m_flyWheel.setSpeedWithSolution(m_shootingMechanism::getShootingSolution);
  }

  /**
   * Starts the flywheel using the shooting solution's computed speed.
   *
   * @return Command that sets flywheel speed from the shooting solution
   */
  public Command setTurretToZero() {
    return m_turret.holdCertainPosition(Degrees.of(0));
  }

  /**
   * Sets the turret to the zero (stowed) position.
   *
   * @return Command that sets turret to zero
   */
  public Command lowerHoodSlowly() {
    return m_hood.moveAngleDownCommand();
  }

  /**
   * Lowers the hood slowly using the hood's slow movement command.
   *
   * @return Command that lowers the hood slowly
   */
  public Command raiseHoodSlowly() {
    return m_hood.moveAngleUpCommand();
  }

  /**
   * Raises the hood slowly using the hood's slow movement command.
   *
   * @return Command that raises the hood slowly
   */
  public Command rotateTurretWithStickInput(Supplier<Double> stick) {
    return m_turret.manualRotationWithSticks(stick);
  }

  /**
   * Creates a command that rotates the turret using direct stick input.
   *
   * @param stick supplier of raw stick input
   * @return Command that applies stick input to turret rotation
   */
  public Command rotateHoodWithStickInput(Supplier<Double> stick) {
    return m_hood.manualRotationWithSticks(stick);
  }

  /**
   * Creates a command that rotates the hood using direct stick input.
   *
   * @param stick supplier of raw stick input
   * @return Command that applies stick input to hood rotation
   */
  public Command raiseIntake() {
    return m_intakeDeployer.raiseIntakeUpCommand();
  }

  /**
   * Raises the intake to the stowed position.
   *
   * @return Command that raises the intake
   */
  public Command lowerIntake() {
    return m_intakeDeployer.lowerIntakeDownCommand();
  }

  /**
   * Lowers the intake to the down position.
   *
   * @return Command that lowers the intake
   */
  public Command agitateIntake() {
    return m_intakeDeployer.agitateIntakeCo();
  }

  /**
   * Runs the intake deployer agitation routine.
   *
   * @return Command that agitates the intake deployer
   */
  public Command startIntake() {
    return m_intake.runIntake();
  }

  /**
   * Starts the intake rollers (alias for {@link #intake()}).
   *
   * @return Command that runs the intake
   */
  public Command reverseIntake() {
    return m_intake.runOuttake();
  }

  /**
   * Reverses the intake rollers to expel game pieces.
   *
   * @return Command that runs the intake in reverse
   */
  public Command stopIntake() {
    return m_intake.stopIntake();
  }

  /**
   * Stops the intake rollers.
   *
   * @return Command that stops the intake
   */
  public Command setIntakeDown() {
    return m_intakeDeployer.lowerIntakeDownCommand();
  }

  /**
   * Lowers the intake (alias for {@link #lowerIntake()}).
   *
   * @return Command that lowers the intake
   */
  public Command zeroTurret() {
    return m_turret.setZero();
  }

  /**
   * Zeros the turret encoder/position.
   *
   * @return Command that sets the turret zero reference
   */
  public Command incrementTurretOffset(double offsetDegrees) {
    return m_turret.adjustPositionDynamically(offsetDegrees);
  }

  /**
   * Adjusts the turret offset dynamically by a specified number of degrees.
   *
   * @param offsetDegrees offset in degrees to apply to the turret position
   * @return Command that adjusts turret offset
   */
  public Command agitateIndexer() {
    return m_indexer.agitateIndexer();
  }

  /**
   * Runs the indexer agitation routine to free jams or move pieces.
   *
   * @return Command that agitates the indexer
   */
  public Command zeroHood() {
    return m_hood.lowerHoodUntilZero();
  }

  /**
   * Lowers the hood until it reaches its zero reference.
   *
   * @return Command that zeros the hood
   */
}
