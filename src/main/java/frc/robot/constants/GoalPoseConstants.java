package frc.robot.constants;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Distance;

public class GoalPoseConstants {
  public static class GoalPose {
    public final Pose2d pose;
    public final boolean requiresShift;
    public final Distance verticalOffset;

    public GoalPose(Pose2d pose, boolean requiresShift, Distance verticalOffset) {
      this.pose = pose;
      this.requiresShift = requiresShift;
      this.verticalOffset = verticalOffset;
    }
  }

  public static final GoalPose BLUE_HUB =
      new GoalPose(WaypointConstants.blueHub, true, Meters.of(1.8288));
  public static final GoalPose RED_HUB =
      new GoalPose(WaypointConstants.redHub, true, Meters.of(1.8288));
  public static final GoalPose RED_RIGHT_SIDE =
      new GoalPose(WaypointConstants.redRightSide, false, Meters.of(0));
  public static final GoalPose RED_LEFT_SIDE =
      new GoalPose(WaypointConstants.redLeftSide, false, Meters.of(0));
  public static final GoalPose BLUE_RIGHT_SIDE =
      new GoalPose(WaypointConstants.blueRightSide, false, Meters.of(0));
  public static final GoalPose BLUE_LEFT_SIDE =
      new GoalPose(WaypointConstants.blueLeftSide, false, Meters.of(0));
}
