package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class WaypointConstants {

  // Debug
  public static final Pose2d TopRightCorner = new Pose2d(6, 2, Rotation2d.fromDegrees(55));
  public static final Pose2d TopLeftCorner = new Pose2d(6, 6, Rotation2d.fromDegrees(55));
  public static final Pose2d BottomRightCorner = new Pose2d(2, 2, Rotation2d.fromDegrees(55));
  public static final Pose2d BottomLeftCorner = new Pose2d(2, 6, Rotation2d.fromDegrees(55));

  @Deprecated
  public static final Pose2d TopRightCornner = TopRightCorner;
  @Deprecated
  public static final Pose2d TopLeftCornner = TopLeftCorner;
  @Deprecated
  public static final Pose2d BottomRightCornner = BottomRightCorner;
  @Deprecated
  public static final Pose2d BottomLeftCornner = BottomLeftCorner;
  public static final Pose2d nearDepotPose = new Pose2d(2, 6, Rotation2d.fromDegrees(55));
  public static final Pose2d nearOutpost = new Pose2d(2.5, 2, Rotation2d.fromDegrees(116));
  public static final Pose2d nearHub = new Pose2d(3, 3.5, Rotation2d.fromDegrees(135));

  public static final Pose2d blueHub = new Pose2d(4.5, 4, Rotation2d.fromDegrees(0));
  public static final Pose2d redHub = new Pose2d(12, 4, Rotation2d.fromDegrees(0));
}
