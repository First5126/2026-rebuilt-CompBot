package frc.robot.vision;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static frc.robot.constants.AprilTagLocalizationConstants.FIELD_LAYOUT;
import static frc.robot.constants.AprilTagLocalizationConstants.LOCALIZATION_PERIOD;
import static frc.robot.constants.AprilTagLocalizationConstants.MAX_TAG_DISTANCE;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngle;
import frc.robot.RobotLogger;
import frc.robot.constants.AprilTagLocalizationConstants.LimelightDetails;
import frc.robot.constants.AprilTagLocalizationConstants.PhotonDetails;
import frc.robot.vision.LimelightHelpers.PoseEstimate;
import java.util.Optional;
import java.util.function.Supplier;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonPipelineResult;

/**
 * Pure logic for AprilTag-based localization.
 *
 * <p>This is split out from {@link AprilTagLocalization} so unit tests can run without requiring
 * WPILib HAL JNI (which is used by {@code SubsystemBase}).
 */
final class AprilTagLocalizationLogic {
  private static final RobotLogger logger = new RobotLogger("AprilTagLocalization");

  @FunctionalInterface
  interface VisionConsumer {
    void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }

  @FunctionalInterface
  interface ResetPose {
    void accept(Pose2d pose2d);
  }

  private final LimelightDetails[] limelightDetails;
  private final PhotonDetails[] photonVisionCameras;
  private final Supplier<Pose2d> robotPoseSupplier;
  private final VisionConsumer visionConsumer;
  private final ResetPose poseReset;

  private boolean fullTrust;
  private final MutAngle yaw = Degrees.mutable(0);
  private final MutAngle oldYaw = Degrees.mutable(0);

  AprilTagLocalizationLogic(
      Supplier<Pose2d> poseSupplier,
      ResetPose resetPose,
      VisionConsumer visionConsumer,
      PhotonDetails[] photonDetails,
      LimelightDetails... limelightDetails) {
    this.limelightDetails = limelightDetails;
    this.photonVisionCameras = photonDetails;
    this.robotPoseSupplier = poseSupplier;
    this.poseReset = resetPose;
    this.visionConsumer = visionConsumer;
  }

  void setFullTrust(boolean fullTrust) {
    this.fullTrust = fullTrust;
  }

  /** Estimates the pose of the robot using AprilTag sources. */
  void poseEstimate() {
    final Pose2d robotPose = robotPoseSupplier.get();
    final double robotYawDegrees = robotPose.getRotation().getDegrees();
    final double maxTagDistanceMeters = MAX_TAG_DISTANCE.in(Meters);

    for (LimelightDetails limelightDetail : limelightDetails) {
      yaw.mut_replace(Degrees.of(robotYawDegrees));
      AngularVelocity yawRate = (yaw.minus(oldYaw).div(LOCALIZATION_PERIOD));

      LimelightHelpers.SetRobotOrientation(
          limelightDetail.name, yaw.in(Degrees), yawRate.in(DegreesPerSecond), 0, 0, 0, 0);

      PoseEstimate poseEstimate =
          LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightDetail.name);
      logger.log(
          "Valid Pose Estimation: ",
          poseEstimate != null
              && poseEstimate.pose.getX() != 0.0
              && poseEstimate.pose.getY() != 0.0);

      if (poseEstimate != null
          && poseEstimate.pose.getX() != 0.0
          && poseEstimate.pose.getY() != 0.0) {
        double scale = poseEstimate.avgTagDist / maxTagDistanceMeters;
        logger.log("Pose Estimate X:", poseEstimate.pose.getX());
        logger.log("Pose Estimate Y:", poseEstimate.pose.getY());

        if (fullTrust) {
          poseReset.accept(
              new Pose2d(
                  poseEstimate.pose.getX(),
                  poseEstimate.pose.getY(),
                  Rotation2d.fromDegrees(yaw.in(Degrees))));
        } else if (!isPoseOffField(poseEstimate.pose)
            && poseEstimate.avgTagDist < maxTagDistanceMeters) {
          Matrix<N3, N1> interpolated =
              interpolate(limelightDetail.closeStdDevs, limelightDetail.farStdDevs, scale);
          visionConsumer.accept(poseEstimate.pose, poseEstimate.timestampSeconds, interpolated);
        }

        oldYaw.mut_replace(yaw);
      }
    }

    for (PhotonDetails photonDetail : photonVisionCameras) {
      PhotonPipelineResult result = photonDetail.camera.getLatestResult();
      Optional<EstimatedRobotPose> estimation =
          photonDetail.poseEstimator.estimateCoprocMultiTagPose(result);

      if (estimation.isEmpty()) {
        estimation = photonDetail.poseEstimator.estimateLowestAmbiguityPose(result);
      }
      final var finalEstimation = estimation;
      estimation.ifPresent(
          est -> {
            double scale =
                PhotonVisionHelpers.getAverageDistanceBetweenTags(
                        photonDetail, finalEstimation.get().estimatedPose.toPose2d())
                    / maxTagDistanceMeters;
            Matrix<N3, N1> interpolated =
                interpolate(photonDetail.closeStdDevs, photonDetail.farStdDevs, scale);

            visionConsumer.accept(est.estimatedPose.toPose2d(), est.timestampSeconds, interpolated);
          });
    }
  }

  private static boolean isPoseOffField(Pose2d observation) {
    return observation.getX() < 0.0
        || observation.getY() < 0.0
        || observation.getX() > FIELD_LAYOUT.getFieldLength()
        || observation.getY() > FIELD_LAYOUT.getFieldWidth();
  }

  private static Matrix<N3, N1> interpolate(
      Matrix<N3, N1> closeStdDevs, Matrix<N3, N1> farStdDevs, double scale) {
    return VecBuilder.fill(
        MathUtil.interpolate(closeStdDevs.get(0, 0), farStdDevs.get(0, 0), scale),
        MathUtil.interpolate(closeStdDevs.get(1, 0), farStdDevs.get(1, 0), scale),
        MathUtil.interpolate(closeStdDevs.get(2, 0), farStdDevs.get(2, 0), scale));
  }
}
