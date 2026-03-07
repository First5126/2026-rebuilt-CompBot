package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.constants.AprilTagLocalizationConstants;
import frc.robot.constants.AprilTagLocalizationConstants.PhotonDetails;
import java.util.List;
import java.util.Optional;
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
  public static double findDIstance(Pose2d pose1, Pose2d pose2) {

    return PhotonUtils.getDistanceToPose(pose1, pose2);
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
    Optional<Pose3d> currentTagPose;
    for (PhotonTrackedTarget target : targets) {
      currentTagPose =
          AprilTagLocalizationConstants.FIELD_LAYOUT.getTagPose(target.getFiducialId());
      if (currentTagPose.isPresent()) {
        totalDistanceOfTargets +=
            PhotonUtils.getDistanceToPose(robotPose2d, currentTagPose.get().toPose2d());
      }
    }
    return totalDistanceOfTargets / numberOfTargets;
  }
}
