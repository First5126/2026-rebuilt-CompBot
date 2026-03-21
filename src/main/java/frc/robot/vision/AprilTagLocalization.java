// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FMS.Zones;
import frc.robot.constants.AprilTagLocalizationConstants.LimelightDetails;
import frc.robot.constants.AprilTagLocalizationConstants.PhotonDetails;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.function.Supplier;

/**
 * A class that uses the limelight to localize the robot using AprilTags. it runs in a background
 * thread instead of the main robot loop.
 */
public class AprilTagLocalization extends SubsystemBase {
  private final AprilTagLocalizationLogic m_logic;
  private final Zones m_zone;

  @SuppressWarnings("unused")
  private final CommandSwerveDrivetrain m_drivetrain;

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
    m_drivetrain = drivetrain;
    m_zone = zone;
    m_logic =
        new AprilTagLocalizationLogic(
            poseSupplier, resetPose::accept, visionConsumer::accept, photonDetails, details);
  }

  /**
   * Sets the full trust of the vision system. The robot will trust the vision system over all other
   * sensors.
   *
   * @param fullTrust true to trust vision fully
   */
  public void setFullTrust(boolean fullTrust) {
    m_logic.setFullTrust(fullTrust);
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
   * Estimates the pose of the robot using the limelight. This function will run in a background
   * thread once per AprilTagLocalizationConstants.LOCALIZATION_PERIOD.
   */
  public void poseEstimate() {
    m_logic.poseEstimate();
  }

  /**
   * Defines a function pointer to a function with a signature ( Pose2d visionRobotPoseMeters,
   * double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs) to accept the vision pose
   * estimate
   */
  @FunctionalInterface
  public interface VisionConsumer {
    void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }

  @FunctionalInterface
  public interface ResetPose {
    void accept(Pose2d pose2d);
  }

  @Override
  public void periodic() {
    if (m_zone.isNearBump()) {
      return;
    }
    poseEstimate();
  }
}
