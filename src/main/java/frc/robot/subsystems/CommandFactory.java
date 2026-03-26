package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FMS.ShiftData;
import frc.robot.FMS.Zones;
import frc.robot.constants.ControllerConstants.OperatorState;
import frc.robot.constants.WaypointConstants;
import frc.robot.controller.Operator;
import java.util.Set;
import java.util.function.Supplier;

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

  public Command resetFMSTime() {
    return Commands.runOnce(
        () -> {
          ShiftData.resetMatchTimeCalibration();
        });
  }

  // Clear a potential jam by running the flywheel inward while reversing the indexer
  public Command clearShootingJam() {
    return m_flyWheel.shootInCommand().alongWith(m_indexer.reverseIndexing());
  }

  // Stop both the flywheel and the indexer
  public Command stopFlywheelAndIndexer() {
    return m_flyWheel.stopSpinning().alongWith(m_indexer.stopIndexing());
  }

  /* sets the hood position for going under the trench */
  public Command setHoodToTrenchPosition() {
    return m_hood.holdCertainPosition(Degrees.of(0));
  }

  public Command rotateTurretBy(Angle amountOfMovement) {
    return m_turret.manualRotation(amountOfMovement);
  }

  public Command stopEverything() {
    return Commands.runOnce(
        () -> {
          stopFlywheel();
          stopIndexer();
          stopIntake();
        });
  }

  public Command rotateHoodBy(Angle amountOfMovement) {
    return m_hood.manualRotation(amountOfMovement);
  }

  public Command spinFlywheelContinuously() {
    return m_flyWheel.rotateFlywheel();
  }

  public Command stopFlywheel() {
    return m_flyWheel.stopSpinning();
  }

  public Command duckHood() {
    return m_hood.holdCertainPosition(Degrees.of(0));
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
    // Command startFlyWheel = m_flyWheel
    //     .setSpeedWithSolution(m_shootingMechanism::getShootingSolution)
    //     .alongWith(m_hood.setPosition(m_shootingMechanism::getShootingSolution));
    Command startIndex = m_indexer.startIndexing();
    return startIndex;
  }

  public Command stopIndexAndShoot() {
    Command stopIndex = m_indexer.stopIndexing();
    return stopIndex;
  }

  // Stop flywheel and stow hood to 0 degrees
  public Command stopFlywheelAndStowHood() {
    return m_flyWheel.stopSpinning().alongWith(m_hood.setPosition(Degree.of(0)));
  }

  public Command startIndexing() {
    return m_indexer.startIndexing();
  }

  public Command stopIndexing() {
    return m_indexer.stopIndexing();
  }

  private boolean isNormalOperatingState() {
    OperatorState state = Operator.getInstance().getOperatorState();
    return state == null || state == OperatorState.NORMAL;
  }

  public Command startTurretTracking() {
    return Commands.defer(
        () -> {
          if (isNormalOperatingState()) {
            Command turretCommand = m_shootingMechanism.startTrackingCommand();
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

  // Convenience / clearer naming for indexing control
  public Command startIndexer() {
    return startIndexing();
  }

  public Command rotateTurretToZero() {
    return m_turret.holdCertainPosition(Degrees.of(0));
  }

  public Command stopIndexer() {
    return stopIndexing();
  }

  public Command test() {
    return Commands.none();
  }

  public Command startFlywheelWithSolution() {
    return m_flyWheel
        .setSpeedWithSolution(m_shootingMechanism::getShootingSolution);
  }

  public Command setTurretToZero() {
    return m_turret.holdCertainPosition(Degrees.of(0));
  }

  public Command lowerHoodSlowly() {
    return m_hood.moveAngleDownCommand();
  }

  public Command raiseHoodSlowly() {
    return m_hood.moveAngleUpCommand();
  }

  public Command rotateTurretWithStickInput(Supplier<Double> stick) {
    return m_turret.manualRotationWithSticks(stick);
  }

  public Command rotateHoodWithStickInput(Supplier<Double> stick) {
    return m_hood.manualRotationWithSticks(stick);
  }

  public Command raiseIntake() {
    return m_intakeDeployer.raiseIntakeUpCommand();
  }

  public Command lowerIntake() {
    return m_intakeDeployer.lowerIntakeDownCommand();
  }

  public Command agitateIntake() {
    return m_intakeDeployer.agitateIntakeCo();
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

  public Command zeroTurret() {
    return m_turret.setZero();
  }
}
