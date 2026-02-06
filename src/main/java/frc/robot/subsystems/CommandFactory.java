package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.TurretConstants;
import frc.robot.constants.WaypointConstants;
import java.util.Set;
import java.util.function.Supplier;

import javax.swing.text.Position;

public class CommandFactory {

  private CommandSwerveDrivetrain m_drivetrain;
  private int m_side = 1;
  private Turret m_turret;
  private Hood m_hood;
  private FuelRollers m_fuelRollers;
  private Intake m_intake;

  public CommandFactory(CommandSwerveDrivetrain drivetrain
  //Turret turret, 
  //Hood hood, 
  //FuelRollers fuelRollers, 
  //Intake intake
  ) {
    //m_turret = turret;
    //m_hood = hood;
    //m_fuelRollers = fuelRollers;
    //m_intake = intake;
    this.m_drivetrain = drivetrain;
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

  /* 

  public Command turretRotateToPosition(Angle position) {
    return Commands.runOnce(
        () -> {
          m_turret.setPosition(position);
          ;
        });
  }


  public Command turretTrackTargetPose(Supplier<Pose2d> robotCurrentPose, Supplier<Pose2d> targetPose) {
    return Commands.run(
        () -> {
          if (robotCurrentPose != null && targetPose.get() != null) {
            Pose2d turretPose = robotCurrentPose.get().plus(TurretConstants.TURRET_OFFSET);

            double distanceX = targetPose.get().getX() - turretPose.getX();
            double distanceY = targetPose.get().getY() - turretPose.getY();

            Rotation2d fieldRelativeAngle =
                Rotation2d.fromRadians(Math.atan2(distanceY, distanceX));

            Rotation2d robotRelativeAngle =
                fieldRelativeAngle.minus(robotCurrentPose.get().getRotation());
                
            m_turret.setPosition(robotRelativeAngle.getMeasure());
          }
        });
  }


  public Command setHoodPosition(Angle position) {
    return Commands.runOnce(
        () -> {
          m_turret.setPosition(position);
        });
  }
  */

}
