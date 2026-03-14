// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.FMS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.GoalPoseConstants;
import frc.robot.constants.GoalPoseConstants.GoalPose;
import frc.robot.constants.ZonesConstants;
import frc.robot.constants.ZonesConstants.Bump;
import frc.robot.constants.ZonesConstants.Trench;
import frc.robot.constants.ZonesConstants.Zone;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.Optional;
import java.util.function.Supplier;

/** Add your docs here. */
public class Zones {
  private static Supplier<Pose2d> m_pose;

  private Optional<Alliance> m_team;
  private CommandSwerveDrivetrain m_commandSwerveDrivetrain;

  public Zones(CommandSwerveDrivetrain commandSwerveDrivetrain) {
    m_team = DriverStation.getAlliance();
    m_pose = () -> commandSwerveDrivetrain.getPose2d();
    m_commandSwerveDrivetrain = commandSwerveDrivetrain;
  }

  public Zone getZone() {
    Translation2d robotTranslation = m_pose.get().getTranslation();
    Zone zone =
        ZonesConstants.firstContainingOrDefault(robotTranslation, Zone.class, Zone.OUT_OF_BOUNDS);
    SmartDashboard.putString("CurrentZone", zone.name());
    return zone;
  }

  public boolean isNearBump() {
    Translation2d robotTranslation = m_pose.get().getTranslation();
    boolean isNearBump = ZonesConstants.containsAny(robotTranslation, Bump.class);
    SmartDashboard.putBoolean("Zones/IsNearBump", isNearBump);
    return isNearBump;
  }

  public boolean isNearTrench() {
    Pose2d robotPose = m_commandSwerveDrivetrain.getPredictedPose2d(0.25);
    Translation2d robotTranslation = robotPose.getTranslation();
    boolean isNearTrench = ZonesConstants.containsAny(robotTranslation, Trench.class);
    SmartDashboard.putBoolean("Zones/IsNearTrench", isNearTrench);
    return isNearTrench;
  }

  public boolean isInDeadZone() {
    Translation2d robotTranslation = m_pose.get().getTranslation();
    // TODO: need to check if in red aliance that the deadzone for shoowing is
    // behind the red alliance wall and vice versa for blue
    // SmartDashboard.putBoolean("Drive/IsInDeadZone", isInDeadZone);
    // return isInDeadZone;
    return false;
  }

  public Pose2d getTurretShootingPose() {
    return getGoalPose().pose;
  }

  public GoalPose getGoalPose() {
    if (!m_team.isPresent()) {
      return GoalPoseConstants.BLUE_HUB;
    }
    switch (getZone()) {
      case ALLIANCE_ZONE:
        return m_team.get() == Alliance.Blue
            ? GoalPoseConstants.BLUE_HUB
            : GoalPoseConstants.RED_HUB;
      case NEUTRAL_ZONE_LEFT:
        return m_team.get() == Alliance.Blue
            ? GoalPoseConstants.BLUE_LEFT_SIDE
            : GoalPoseConstants.RED_LEFT_SIDE;
      case NEUTRAL_ZONE_RIGHT:
        return m_team.get() == Alliance.Blue
            ? GoalPoseConstants.BLUE_RIGHT_SIDE
            : GoalPoseConstants.RED_RIGHT_SIDE;
      case OPPONENT_ZONE:
        return m_team.get() == Alliance.Blue
            ? GoalPoseConstants.BLUE_HUB
            : GoalPoseConstants.RED_HUB;
      default:
        break;
    }
    return m_team.get() == Alliance.Blue ? GoalPoseConstants.BLUE_HUB : GoalPoseConstants.RED_HUB;
  }
}
