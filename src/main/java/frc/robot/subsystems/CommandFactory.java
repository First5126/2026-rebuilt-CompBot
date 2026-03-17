package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.FMS.ShiftData;
import frc.robot.FMS.Zones;
import frc.robot.constants.ControllerConstants.OperatorState;
import frc.robot.constants.WaypointConstants;
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
  private OperatorState operatorState;
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
      OperatorState operatorState,
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
    this.operatorState = operatorState;
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

  public Command reverseShootingCommand() {
    return m_flyWheel.shootInCommand().alongWith(m_indexer.reverseIndexing());
  }

  public Command stopShootingCommand() {
    return m_flyWheel.stopSpinning().alongWith(m_indexer.stopIndexing());
  }

  /* tells the hood to duck for going under the trench */
  public Command duckHood() {
    return m_hood.holdCertainPosition(Degrees.of(0));
  }

  public Command manualTurretRotation(Angle amountOfMovement) {
    return m_turret.manualRotation(amountOfMovement);
  }

  public Command manualHoodRotation(Angle amountOfMovement) {
    return m_hood.manualRotation(amountOfMovement);
  }

  public Command rotateFlywheel() {
    return m_flyWheel.rotateFlywheel();
  }

  public Command stopShooting() {
    return m_flyWheel.stopSpinning();
  }

  public ConditionalCommand startShootingMechanism() {
    return new ConditionalCommand(
        m_flyWheel
            .setSpeedWithSolution(m_shootingMechanism::getShootingSolution)
            .alongWith(m_hood.setPosition(m_shootingMechanism::getShootingSolution)),
        Commands.none(),
        this::checkIfNotOverride);
  }

  public Command stopShootingMechanism() {
    return m_flyWheel.stopSpinning().alongWith(m_hood.setPosition(Degree.of(0)));
  }

  public Command startIndexing() {
    return m_indexer.startIndexing();
  }

  public Command stopIndexing() {
    return m_indexer.stopIndexing();
  }

  private boolean checkIfNotOverride() {
    return operatorState == OperatorState.NORMAL;
  }

  public ConditionalCommand startTurretTracking() {
    return new ConditionalCommand(
        m_shootingMechanism.startTrackingCommand(), Commands.none(), this::checkIfNotOverride);
  }

  public Command shootCommand() {
    return startIndexing();
  }

  public Command stopShootCommand() {
    return stopIndexing();
  }

  public Command startFlywheelsWithSolution() {
    return m_flyWheel.setSpeedWithSolution(m_shootingMechanism::getShootingSolution);
  }

  public Command slowlyMoveHoodDown() {
    return m_hood.moveAngleDownCommand();
  }

  public Command slowlyMoveHoodUp() {
    return m_hood.moveAngleUpCommand();
  }

  public Command moveTurretManualy(Angle angle) {
    return m_turret.manualRotation(angle);
  }

  public Command moveTurretManualyWithSticks(Supplier<Double> stick) {
    return m_turret.manualRotationWithSticks(stick);
  }

  public Command moveHoodManualyWithSticks(Supplier<Double> stick) {
    return m_hood.manualRotationWithSticks(stick);
  }

  public Command raiseIntake() {
    return m_intakeDeployer.raiseIntakeUpCommand();
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
}
