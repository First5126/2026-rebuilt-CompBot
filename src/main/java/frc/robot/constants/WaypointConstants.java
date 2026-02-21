package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class WaypointConstants {

  // Debug
  public static final Pose2d TopRightCornner = new Pose2d(6, 2, Rotation2d.fromDegrees(55));
  public static final Pose2d TopLeftCornner = new Pose2d(6, 6, Rotation2d.fromDegrees(55));
  public static final Pose2d BottomRightCornner = new Pose2d(2, 2, Rotation2d.fromDegrees(55));
  public static final Pose2d BottomLeftCornner = new Pose2d(2, 6, Rotation2d.fromDegrees(55));

  public static final Pose2d nearDepotPose = new Pose2d(2, 6, Rotation2d.fromDegrees(55));
  public static final Pose2d nearOutpost = new Pose2d(2.5, 2, Rotation2d.fromDegrees(116));
  public static final Pose2d nearHub = new Pose2d(3, 3.5, Rotation2d.fromDegrees(135));

  public static final Pose2d blueHub = new Pose2d(4.625, 4.025, Rotation2d.fromDegrees(0));
  public static final Pose2d redHub = new Pose2d(11.925, 4.025, Rotation2d.fromDegrees(0));

  public static final Pose2d redRightSide = new Pose2d(14, 2, Rotation2d.fromDegrees(0));
  public static final Pose2d redLeftSide = new Pose2d(14, 6, Rotation2d.fromDegrees(0));
  public static final Pose2d blueRightSide = new Pose2d(2, 2, Rotation2d.fromDegrees(0));
  public static final Pose2d blueLeftSide = new Pose2d(2, 6, Rotation2d.fromDegrees(0));
}
