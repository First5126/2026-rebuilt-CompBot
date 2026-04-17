// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FMS.Zones;
import frc.robot.RobotLogger;
import frc.robot.constants.AprilTagLocalizationConstants.LimelightDetails;
import frc.robot.constants.AprilTagLocalizationConstants.PhotonDetails;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.vision.LimelightHelpers.PoseEstimate;
import java.util.Optional;
import java.util.function.Supplier;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonPipelineResult;

/**
 * A class that uses the limelight to localize the robot using AprilTags. it runs in a background
 * thread instead of the main robot loop.
 */
public class AprilTagLocalization extends SubsystemBase {
  private static final RobotLogger logger = new RobotLogger("AprilTagLocalization");
  private static final int LOG_EVERY_N_ESTIMATES = 25; // ~0.5s if estimates run at 20ms
  private LimelightDetails[] m_LimelightDetails; // list of limelights that can provide updates
  private PhotonDetails[] m_PhotonVisionCameras; // list of limelights that can provide updates
  private double[]
      m_lastPhotonTimestamps; // track last processed timestamp per photon camera to avoid duplicate
  // work
  // Cache last computed interpolation scale & stddevs per limelight to avoid repeated Matrix
  // allocations when scale does not change between estimates
  private double[] m_lastLimelightScale;

  @SuppressWarnings("rawtypes")
  private Matrix[] m_lastLimelightInterpolated;

  // Cache last computed interpolation scale & stddevs per photon camera
  private double[] m_lastPhotonScale;

  @SuppressWarnings("rawtypes")
  private Matrix[] m_lastPhotonInterpolated;

  private Supplier<Pose2d> m_robotPoseSupplier; // supplies the pose of the robot
  private boolean m_FullTrust; // to allow for button trust the tag estimate over all else.
  private MutAngle m_yaw = Degrees.mutable(0);
  CommandSwerveDrivetrain m_drivetrain;
  private MutAngle m_OldYaw = Degrees.mutable(0); // the previous yaw
  private VisionConsumer m_VisionConsumer;
  private ResetPose m_poseReset;
  private Zones m_zone;
  private int m_logCountdown = 0;
  private int updateCounter = 1;

  /**
   * Creates a new AprilTagLocalization.
   *
   * @param poseSupplier supplies the current robot pose
   * @param resetPose consumer to reset the drivetrain pose
   * @param visionConsumer a consumer that accepts the vision pose, timestamp, and standard
   *     deviations
   * @param drivetrain drivetrain used for reference and pose access
   * @param photonDetails details for PhotonVision cameras
   * @param details the details of the limelight; more than one can be passed to allow for multiple
   *     on the bot.
   */
  public AprilTagLocalization(
      Supplier<Pose2d> poseSupplier,
      ResetPose resetPose,
      VisionConsumer visionConsumer,
      CommandSwerveDrivetrain drivetrain,
      Zones zone,
      PhotonDetails[] photonDetails,
      LimelightDetails... details) {
    m_LimelightDetails = details;
    m_PhotonVisionCameras = photonDetails;
    if (m_PhotonVisionCameras != null) {
      m_lastPhotonTimestamps = new double[m_PhotonVisionCameras.length];
      for (int i = 0; i < m_lastPhotonTimestamps.length; i++) {
        m_lastPhotonTimestamps[i] = 0.0;
      }
    } else {
      m_lastPhotonTimestamps = new double[0];
    }
    // Initialize interpolation caches for limelights
    if (m_LimelightDetails != null) {
      m_lastLimelightScale = new double[m_LimelightDetails.length];
      // generic array creation - acceptable here for caching
      m_lastLimelightInterpolated = new Matrix[m_LimelightDetails.length];
      for (int i = 0; i < m_lastLimelightScale.length; i++) {
        m_lastLimelightScale[i] = Double.NaN;
        m_lastLimelightInterpolated[i] = null;
      }
    } else {
      m_lastLimelightScale = new double[0];
      m_lastLimelightInterpolated = new Matrix[0];
    }
    // Initialize interpolation caches for photon cameras
    if (m_PhotonVisionCameras != null) {
      m_lastPhotonScale = new double[m_PhotonVisionCameras.length];
      m_lastPhotonInterpolated = new Matrix[m_PhotonVisionCameras.length];
      for (int i = 0; i < m_lastPhotonScale.length; i++) {
        m_lastPhotonScale[i] = Double.NaN;
        m_lastPhotonInterpolated[i] = null;
      }
    } else {
      m_lastPhotonScale = new double[0];
      m_lastPhotonInterpolated = new Matrix[0];
    }
    m_robotPoseSupplier = poseSupplier;
    m_poseReset = resetPose;
    m_VisionConsumer = visionConsumer;
    m_drivetrain = drivetrain;
    m_zone = zone;
  }

  /**
   * Sets the full trust of the vision system. The robot will trust the vision system over all other
   * sensors.
   *
   * @param fullTrust true to trust vision fully
   */
  public void setFullTrust(boolean fullTrust) {
    m_FullTrust = fullTrust;
  }

  /**
   * Creates a command to toggle full-trust mode.
   *
   * @param trust true to enable full trust, false to disable
   * @return command that applies the trust setting once
   */
  public Command setTrust(boolean trust) {
    return Commands.runOnce(
        () -> {
          setFullTrust(trust);
        });
  }

  /**
   * Checks if the pose is off the field.
   *
   * @param observation
   * @return
   */
  private boolean isPoseOffField(Pose2d observation) {
    // Coordinates for where Pose is on the field
    return observation.getX() < 0.0 // What X position robot is on the field.
        || observation.getY() < 0.0 // What Y position robot is on the field.
        || observation.getX()
            > FIELD_LAYOUT.getFieldLength() // Whether the robot X position is on the field or not
        || observation.getY()
            > FIELD_LAYOUT.getFieldWidth(); // Whether the robot X position is on the field or not
  }

  /**
   * Interpolates between two std deviations.
   *
   * @param closeStdDevs
   * @param farStdDevs
   * @param scale
   * @return
   */
  private Matrix<N3, N1> interpolate(
      Matrix<N3, N1> closeStdDevs, Matrix<N3, N1> farStdDevs, double scale) {
    return VecBuilder.fill(
        MathUtil.interpolate(closeStdDevs.get(0, 0), farStdDevs.get(0, 0), scale),
        MathUtil.interpolate(closeStdDevs.get(1, 0), farStdDevs.get(1, 0), scale),
        MathUtil.interpolate(closeStdDevs.get(2, 0), farStdDevs.get(2, 0), scale));
  }

  /**
   * Estimates the pose of the robot using the limelight. This function will run in a background
   * thread once per AprilTagLocalizationConstants.LOCALIZATION_PERIOD.
   */
  public void poseEstimate() {
    final boolean shouldLog = (m_logCountdown-- <= 0);
    if (shouldLog) {
      m_logCountdown = LOG_EVERY_N_ESTIMATES;
    }

    final Pose2d robotPose = m_robotPoseSupplier.get();
    final double robotYawDegrees = robotPose.getRotation().getDegrees();
    final double maxTagDistanceMeters = MAX_TAG_DISTANCE.in(Meters);

    for (int li = 0; li < m_LimelightDetails.length; li++) {
      LimelightDetails limelightDetail = m_LimelightDetails[li];
      m_yaw.mut_replace(Degrees.of(robotYawDegrees));
      AngularVelocity yawRate = (m_yaw.minus(m_OldYaw).div(LOCALIZATION_PERIOD));
      // Set Orientation using LimelightHelpers.SetRobotOrientation and the m_robotPoseSupplier
      LimelightHelpers.SetRobotOrientation(
          limelightDetail.name,
          m_yaw.in(Degrees),
          yawRate.in(DegreesPerSecond),
          0,
          0,
          0,
          0); // Set Orientation using LimelightHelpers.SetRobotOrientation and the
      // m_robotPoseSupplier

      // Check if we have at least two tags
      // if (LimelightHelpers.getTargetCount(limelightDetail.name) < 2) {
      //   continue;
      // }

      // Get the pose from the Limelight
      PoseEstimate poseEstimate =
          LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(
              limelightDetail.name); // Get the pose from the Limelight

      if (poseEstimate == null) {
        continue;
      }

      if (shouldLog) {
        logger.logAndDisplay(
            "Valid Pose Estimation: ",
            poseEstimate.pose.getX() != 0.0 && poseEstimate.pose.getY() != 0.0);
      }

      if (poseEstimate.pose.getX() != 0.0 && poseEstimate.pose.getY() != 0.0) {
        // remove the offset of the camera
        /*poseEstimate.pose =
        poseEstimate.pose.transformBy(
        	new Transform2d(
        		limelightDetail.inverseOffset.get(0, 0),
        		limelightDetail.inverseOffset.get(1, 0),
        		Rotation2d.fromDegrees(limelightDetail.inverseOffset.get(2, 0))));*/

        double scale =
            poseEstimate.avgTagDist
                / maxTagDistanceMeters; // scale the std deviation by the distance
        // Validate the pose for sanity; reject bad poses. If fullTrust is true accept regardless.
        if (m_FullTrust) {
          // set the pose in the pose consumer
          m_poseReset.accept(
              new Pose2d(
                  poseEstimate.pose.getX(),
                  poseEstimate.pose.getY(),
                  Rotation2d.fromDegrees(m_yaw.in(Degrees))));
        } else if (!(isPoseOffField(poseEstimate.pose))
            && poseEstimate.avgTagDist
                < maxTagDistanceMeters) { // reject poses that are more than max tag distance we
          // trust
          // Compute or reuse cached interpolated stddevs (only when needed)
          Matrix<N3, N1> interpolated;
          double last = m_lastLimelightScale[li];
          if (Double.isNaN(last) || Math.abs(last - scale) > 1e-6) {
            interpolated =
                interpolate(limelightDetail.closeStdDevs, limelightDetail.farStdDevs, scale);
            m_lastLimelightInterpolated[li] = interpolated;
            m_lastLimelightScale[li] = scale;
          } else {
            @SuppressWarnings("unchecked")
            Matrix<N3, N1> cached = (Matrix<N3, N1>) m_lastLimelightInterpolated[li];
            interpolated = cached;
          }

          // set the pose in the pose consumer
          m_VisionConsumer.accept(poseEstimate.pose, poseEstimate.timestampSeconds, interpolated);
        }
        m_OldYaw.mut_replace(m_yaw);
      }
    }

    for (int i = 0; i < m_PhotonVisionCameras.length; i++) {
      PhotonDetails photonDetail = m_PhotonVisionCameras[i];
      if (photonDetail == null || photonDetail.camera == null) {
        continue;
      }

      PhotonPipelineResult result =
          frc.robot.vision.PhotonVisionHelpers.getResultOfCamera(photonDetail.camera);

      // Fast path checks: must have targets and be newer than last processed timestamp
      double resultTs = result.getTimestampSeconds();
      if (!result.hasTargets() || resultTs <= m_lastPhotonTimestamps[i]) {
        continue;
      }

      // Mark this timestamp as processed up-front to avoid reprocessing the same frame
      m_lastPhotonTimestamps[i] = resultTs;

      Optional<EstimatedRobotPose> estimation =
          photonDetail.poseEstimator.estimateCoprocMultiTagPose(result);

      if (estimation.isEmpty()) {
        estimation = photonDetail.poseEstimator.estimateLowestAmbiguityPose(result);
      }

      if (estimation.isPresent()) {
        EstimatedRobotPose est = estimation.get();
        // Reuse the computed Pose2d to avoid allocating it twice
        Pose2d photonPose2d = est.estimatedPose.toPose2d();
        double scale =
            PhotonVisionHelpers.getAverageDistanceBetweenTags(photonDetail, photonPose2d)
                / maxTagDistanceMeters;

        // compute or reuse interpolated stddevs based on distance to avoid repeated allocations
        Matrix<N3, N1> interpolated;
        double lastP = m_lastPhotonScale[i];
        if (Double.isNaN(lastP) || Math.abs(lastP - scale) > 1e-6) {
          interpolated = interpolate(photonDetail.closeStdDevs, photonDetail.farStdDevs, scale);
          m_lastPhotonInterpolated[i] = interpolated;
          m_lastPhotonScale[i] = scale;
        } else {
          @SuppressWarnings("unchecked")
          Matrix<N3, N1> cached = (Matrix<N3, N1>) m_lastPhotonInterpolated[i];
          interpolated = cached;
        }

        // Only pass the measurement once per new result (timestamp guard above)
        m_VisionConsumer.accept(photonPose2d, est.timestampSeconds, interpolated);
      }
    }
  }

  /**
   * Consumer callback interface for accepting vision pose estimates.
   *
   * <p>Implementations receive a vision-derived robot pose, the timestamp of the observation, and
   * the measurement standard deviations as a 3x1 matrix.
   */
  @FunctionalInterface
  public interface VisionConsumer {
    /**
     * Accepts a vision pose estimate and associated measurement properties.
     *
     * @param visionRobotPoseMeters Pose2d estimate in field coordinates
     * @param timestampSeconds observation timestamp in seconds
     * @param visionMeasurementStdDevs 3x1 matrix containing stddevs for [x, y, theta]
     */
    void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }

  /** Functional interface used to reset or set the drivetrain pose from vision. */
  @FunctionalInterface
  public interface ResetPose {
    /**
     * Accepts a Pose2d to reset the robot/drivetrain pose.
     *
     * @param pose2d new pose to apply to the drivetrain
     */
    void accept(Pose2d pose2d);
  }

  @Override
  /**
   * Periodic scheduler hook. Runs pose estimation on a throttled cadence unless the robot is in
   * certain zones (e.g. near the bump) where estimation is skipped.
   */
  public void periodic() {
    if (m_zone.isNearBump()) {
      return;
    }
    if (2 <= updateCounter) {
      updateCounter = 1;
    } else {
      updateCounter++;
      poseEstimate();
    }
  }
}
