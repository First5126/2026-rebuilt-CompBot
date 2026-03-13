package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degree;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FMS.ShiftData;
import frc.robot.FMS.Zones;
import frc.robot.constants.WaypointConstants;
import java.util.Set;

public class CommandFactory {

  private CommandSwerveDrivetrain m_drivetrain;
  private int m_side = 1;
  private Turret m_turret;
  private Zones m_zone;
  private ShootingMechanism m_shootingMechanism;
  private FlyWheel m_flyWheel;
  private Hood m_hood;
  private Indexer m_indexer;

  public CommandFactory(
      CommandSwerveDrivetrain drivetrain,
      Turret turret,
      Zones zone,
      ShootingMechanism m_shootingMechanism,
      FlyWheel flyWheel,
      Hood hood,
      Indexer indexer) {
    this.m_drivetrain = drivetrain;
    this.m_turret = turret;
    this.m_zone = zone;
    this.m_shootingMechanism = m_shootingMechanism;
    this.m_flyWheel = flyWheel;
    this.m_hood = hood;
    this.m_indexer = indexer;
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
    return Commands.runOnce(() -> {
      m_flyWheel.shootInCommand();
    });
  }

  public Command stopShootingCommand() {
    return Commands.runOnce(() -> {
      m_flyWheel.stopSpinning();
    });
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

  public Command startShootingMechanism() {
    return m_flyWheel
        .setSpeedWithSolution(m_shootingMechanism::getShootingSolution)
        .alongWith(m_hood.setPosition(m_shootingMechanism::getShootingSolution));
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
}
