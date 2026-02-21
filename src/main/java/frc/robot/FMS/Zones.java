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
import frc.robot.constants.ZonesConstants.Bump;
import frc.robot.constants.ZonesConstants.Zone;
import java.util.Optional;
import java.util.function.Supplier;

/** Add your docs here. */
public class Zones {
  private static Supplier<Pose2d> m_pose;

  private Optional<Alliance> m_team;

  public Zones(Supplier<Pose2d> robotPoseSupplier) {
    m_team = DriverStation.getAlliance();
    m_pose = robotPoseSupplier;
  }

  public Zone getZone() {
    double x = m_pose.get().getX();
    double y = m_pose.get().getY();

    if (isWithin(
        x,
        y,
        Zone.ALLIANCE_ZONE.getTopLeftTranslation(),
        Zone.ALLIANCE_ZONE.getBottomRightTranslation())) {
      SmartDashboard.putString("CurrentZone", Zone.ALLIANCE_ZONE.name());
      return Zone.ALLIANCE_ZONE;
    } else if (isWithin(
        x,
        y,
        Zone.NEUTRAL_ZONE_RIGHT.getTopLeftTranslation(),
        Zone.NEUTRAL_ZONE_RIGHT.getBottomRightTranslation())) {
      SmartDashboard.putString("CurrentZone", Zone.NEUTRAL_ZONE_RIGHT.name());
      return Zone.NEUTRAL_ZONE_RIGHT;
    } else if (isWithin(
        x,
        y,
        Zone.NEUTRAL_ZONE_LEFT.getTopLeftTranslation(),
        Zone.NEUTRAL_ZONE_LEFT.getBottomRightTranslation())) {
      SmartDashboard.putString("CurrentZone", Zone.NEUTRAL_ZONE_LEFT.name());
      return Zone.NEUTRAL_ZONE_LEFT;
    } else if (isWithin(
        x,
        y,
        Zone.OPPONENT_ZONE.getTopLeftTranslation(),
        Zone.OPPONENT_ZONE.getBottomRightTranslation())) {
      SmartDashboard.putString("CurrentZone", Zone.OPPONENT_ZONE.name());
      return Zone.OPPONENT_ZONE;
    } else {
      // Outside defined zones, handle as needed
      SmartDashboard.putString("CurrentZone", Zone.OUT_OF_BOUNDS.name());
      return Zone.OUT_OF_BOUNDS;
    }
  }

  public boolean onBump() {
    double x = m_pose.get().getX();
    double y = m_pose.get().getY();

    for (Bump bump : Bump.values()) {
      if (isWithin(x, y, bump.getTopLeftTranslation(), bump.getBottomRightTranslation())) {
        return true;
      }
    }

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

  private boolean isWithin(double x, double y, Translation2d corner1, Translation2d corner2) {
    double minX = Math.min(corner1.getX(), corner2.getX());
    double maxX = Math.max(corner1.getX(), corner2.getX());
    double minY = Math.min(corner1.getY(), corner2.getY());
    double maxY = Math.max(corner1.getY(), corner2.getY());

    return (x >= minX && x <= maxX) && (y >= minY && y <= maxY);
  }
}
