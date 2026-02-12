package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.constants.AprilTagLocalizationConstants;
import frc.robot.constants.AprilTagLocalizationConstants.PhotonDetails;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonVisionHelpers {

  /**
   * Retrieves the latest pipeline result from a Photon camera.
   *
   * @param camera photon camera instance
   * @return latest pipeline result
   */
  public static PhotonPipelineResult getResultOfCamera(PhotonCamera camera) {

    return camera.getLatestResult();
  }

  /**
   * Computes the distance between two poses.
   *
   * @param pose1 first pose
   * @param pose2 second pose
   * @return distance in meters between the poses
   */
  public static double findDistance(Pose2d pose1, Pose2d pose2) {

    return PhotonUtils.getDistanceToPose(pose1, pose2);
  }

  /**
   * Computes the distance between two poses.
   *
   * @param pose1 first pose
   * @param pose2 second pose
   * @return distance in meters between the poses
   * @deprecated Use {@link #findDistance(Pose2d, Pose2d)} instead.
   */
  @Deprecated(forRemoval = false)
  public static double findDIstance(Pose2d pose1, Pose2d pose2) {

    return findDistance(pose1, pose2);
  }

  /**
   * Computes the average distance from the robot pose to visible AprilTags.
   *
   * @param photonDetail configured PhotonVision camera details
   * @param robotPose2d current robot pose
   * @return average distance in meters, or 0.0 if no targets are visible
   */
  public static double getAvrageDistanceBetweenTags(
      PhotonDetails photonDetail, Pose2d robotPose2d) {

    // TODO Cole  Replace deprecated getLatestResult
    List<PhotonTrackedTarget> targets = photonDetail.camera.getLatestResult().getTargets();
    int numberOfTargets = targets.size();

    // Return 0.0 when no targets are available to avoid division by zero.
    // This results in a scale factor of 0 in pose estimation, indicating close distance to tags.
    if (numberOfTargets == 0) {
      return 0.0;
    }

    double totalDistanceOfTargets = 0;
    for (PhotonTrackedTarget target : targets) {
      totalDistanceOfTargets +=
          PhotonUtils.getDistanceToPose(
              robotPose2d,
              AprilTagLocalizationConstants.FIELD_LAYOUT
                  .getTagPose(target.getFiducialId())
                  .orElseThrow(
                      () ->
                          new IllegalArgumentException(
                              "No tag pose found in FIELD_LAYOUT for fiducial ID "
                                  + target.getFiducialId()))
                  .toPose2d());
    }
    return totalDistanceOfTargets / numberOfTargets;
  }
}
