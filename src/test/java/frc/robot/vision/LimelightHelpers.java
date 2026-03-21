package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

/**
 * Test stub that shadows the main LimelightHelpers at test runtime, letting tests drive expected
 * behavior deterministically without NetworkTables.
 */
public class LimelightHelpers {
  private static final Map<String, PoseEstimate> poseEstimates = new ConcurrentHashMap<>();

  public static volatile OrientationCall lastOrientationCall = null;

  public static void setPoseEstimate(String limelightName, PoseEstimate estimate) {
    poseEstimates.put(limelightName, estimate);
  }

  public static void clear() {
    poseEstimates.clear();
    lastOrientationCall = null;
  }

  public static void SetRobotOrientation(
      String limelightName,
      double yaw,
      double yawRate,
      double pitch,
      double pitchRate,
      double roll,
      double rollRate) {
    lastOrientationCall =
        new OrientationCall(limelightName, yaw, yawRate, pitch, pitchRate, roll, rollRate);
  }

  public static PoseEstimate getBotPoseEstimate_wpiBlue_MegaTag2(String limelightName) {
    return poseEstimates.get(limelightName);
  }

  public record OrientationCall(
      String limelightName,
      double yaw,
      double yawRate,
      double pitch,
      double pitchRate,
      double roll,
      double rollRate) {}

  public static class PoseEstimate {
    public Pose2d pose;
    public double timestampSeconds;
    public double latency;
    public int tagCount;
    public double tagSpan;
    public double avgTagDist;
    public double avgTagArea;
    public RawFiducial[] rawFiducials;
    public boolean isMegaTag2;

    public PoseEstimate() {
      this.pose = new Pose2d();
      this.timestampSeconds = 0;
      this.latency = 0;
      this.tagCount = 0;
      this.tagSpan = 0;
      this.avgTagDist = 0;
      this.avgTagArea = 0;
      this.rawFiducials = new RawFiducial[] {};
      this.isMegaTag2 = false;
    }

    public PoseEstimate(Pose2d pose, double timestampSeconds, double avgTagDist) {
      this();
      this.pose = pose;
      this.timestampSeconds = timestampSeconds;
      this.avgTagDist = avgTagDist;
    }
  }

  public static class RawFiducial {}
}
