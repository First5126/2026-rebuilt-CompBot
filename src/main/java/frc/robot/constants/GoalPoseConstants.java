package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.constants.ShootingMechanismConstants.InterpolationSet;

public class GoalPoseConstants {
  public static class GoalPose {
    public final Pose2d pose;
    public final boolean requiresShift;
    public final InterpolationSet interpolationSet;

    public GoalPose(Pose2d pose, boolean requiresShift, InterpolationSet interpolationSet) {
      this.pose = pose;
      this.requiresShift = requiresShift;
      this.interpolationSet = interpolationSet;
    }
  }

  public static final GoalPose BLUE_HUB =
      new GoalPose(WaypointConstants.blueHub, true, ShootingMechanismConstants.hubInterpolation);
  public static final GoalPose RED_HUB =
      new GoalPose(WaypointConstants.redHub, true, ShootingMechanismConstants.hubInterpolation);
  public static final GoalPose RED_RIGHT_SIDE =
      new GoalPose(
          WaypointConstants.redRightSide, false, ShootingMechanismConstants.floorInterpolation);
  public static final GoalPose RED_LEFT_SIDE =
      new GoalPose(
          WaypointConstants.redLeftSide, false, ShootingMechanismConstants.floorInterpolation);
  public static final GoalPose BLUE_RIGHT_SIDE =
      new GoalPose(
          WaypointConstants.blueRightSide, false, ShootingMechanismConstants.floorInterpolation);
  public static final GoalPose BLUE_LEFT_SIDE =
      new GoalPose(
          WaypointConstants.blueLeftSide, false, ShootingMechanismConstants.floorInterpolation);
}
