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

  public static PhotonPipelineResult getResultOfCamera(PhotonCamera camera) {

    return camera.getLatestResult();
  }

  public static double findDIstance(Pose2d pose1, Pose2d pose2) {

    return PhotonUtils.getDistanceToPose(pose1, pose2);
  }

  public static double getAvrageDistanceBetweenTags(
      PhotonDetails photonDetail, Pose2d robotPose2d) {

    // TODO Cole  Replace deprecated getLasterdResult
    List<PhotonTrackedTarget> targets = photonDetail.camera.getLatestResult().getTargets();
    int numberOfTargets = targets.size();
    double totalDistanceOfTargets = 0;
    for (PhotonTrackedTarget target : targets) {
      totalDistanceOfTargets += 
          PhotonUtils.getDistanceToPose(
              robotPose2d,
              AprilTagLocalizationConstants.FIELD_LAYOUT
                  .getTagPose(target.getFiducialId())
                  .get()
                  .toPose2d());
    }
    return totalDistanceOfTargets / numberOfTargets;
  }
}
