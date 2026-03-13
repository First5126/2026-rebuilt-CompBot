package frc.robot.subsystems;

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
  private Hood m_hood;
  private FlyWheel m_flyWheel;

  public CommandFactory(
      CommandSwerveDrivetrain drivetrain,
      Turret turret,
      Zones zone,
      ShootingMechanism m_shootingMechanism,
      Hood m_hood,
      FlyWheel flywheel) {
    this.m_drivetrain = drivetrain;
    this.m_turret = turret;
    this.m_zone = zone;
    this.m_shootingMechanism = m_shootingMechanism;
    this.m_hood = m_hood;
    this.m_flyWheel = flywheel;
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
    return Commands.runOnce(() -> {
      ShiftData.resetMatchTimeCalibration();
    });
  }

  public Command moveHoodAngleUp() {
    return Commands.run(() -> {
      m_hood.moveAngleUpCommand();

    });
  }

  public Command moveHoodAngleDown() {
    return Commands.run(() -> {
      m_hood.moveAngleDownCommand();
    });
  }

  public Command moveTurretPosition() {
    return Commands.run(() -> {

    });
  }

  public Command rotateFlywheel() {
    return m_flyWheel.rotateFlywheel();
  }

  public Command stopFlyWheel() {
    return m_flyWheel.stopSpinning();
  }
}