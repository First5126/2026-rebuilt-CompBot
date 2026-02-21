package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;

public class GoalPoseConstants {
  public static class GoalPose {
    public final Pose2d pose;
    public final boolean requiresShift;

    public GoalPose(Pose2d pose, boolean requiresShift) {
      this.pose = pose;
      this.requiresShift = requiresShift;
    }
  }

  public static final GoalPose BLUE_HUB = new GoalPose(WaypointConstants.blueHub, true);
  public static final GoalPose RED_HUB = new GoalPose(WaypointConstants.redHub, true);
  public static final GoalPose RED_RIGHT_SIDE = new GoalPose(WaypointConstants.redRightSide, false);
  public static final GoalPose RED_LEFT_SIDE = new GoalPose(WaypointConstants.redLeftSide, false);
  public static final GoalPose BLUE_RIGHT_SIDE =
      new GoalPose(WaypointConstants.blueRightSide, false);
  public static final GoalPose BLUE_LEFT_SIDE = new GoalPose(WaypointConstants.blueLeftSide, false);
}
