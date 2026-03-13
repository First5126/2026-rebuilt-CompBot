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
  private Intake m_intake;

  public CommandFactory(
      CommandSwerveDrivetrain drivetrain,
      Turret turret,
      Zones zone,
      ShootingMechanism m_shootingMechanism,
      Hood m_hood,
      FlyWheel flywheel,
      Intake intake) {
    this.m_drivetrain = drivetrain;
    this.m_turret = turret;
    this.m_zone = zone;
    this.m_shootingMechanism = m_shootingMechanism;
    this.m_hood = m_hood;
    this.m_flyWheel = flywheel;
    this.m_intake = intake;
  }


  public Command resetFMSTime() {
    return Commands.runOnce(() -> {
      ShiftData.resetMatchTimeCalibration();
    });
  }

  public Command moveHoodAngleUp() {
    return m_hood.moveAngleUpCommand();
  };
  

  public Command moveHoodAngleDown() {
    return m_hood.moveAngleDownCommand();
  };

  public Command runIntakeWheels() {
    return m_intake.runIntakeWheelsCommand();
  };

  public Command runOuttakeWheels() {
    return m_intake.runOuttakeWheelsCommand();
  }




  public Command rotateFlywheel() {
    return m_flyWheel.rotateFlywheel();
  }

  public Command stopFlyWheel() {
    return m_flyWheel.stopSpinning();
  }
}