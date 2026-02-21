// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

/** Add your docs here. */
public class AprilTagLocalizationConstants {
  public static class LimelightDetails {
    public String name;
    public Matrix<N3, N1> closeStdDevs;
    public Matrix<N3, N1> farStdDevs;
    public Matrix<N3, N1> inverseOffset;

    /*
     *
     */
    public LimelightDetails(String name, Matrix<N3, N1> closeStdDevs, Matrix<N3, N1> farStdDevs) {
      this.name = name;
      this.closeStdDevs = closeStdDevs;
      this.farStdDevs = farStdDevs;
    }
  }

  public static class PhotonDetails {
    public PhotonCamera camera;
    public Transform3d robotToCam;
    public Matrix<N3, N1> closeStdDevs;
    public Matrix<N3, N1> farStdDevs;
    public PhotonPoseEstimator poseEstimator;

    /*
     *
     */
    public PhotonDetails(
        PhotonCamera camera,
        Transform3d robotToCam,
        Matrix<N3, N1> closeStdDevs,
        Matrix<N3, N1> farStdDevs) {
      this.camera = camera;
      this.robotToCam = robotToCam;
      this.closeStdDevs = closeStdDevs;
      this.farStdDevs = farStdDevs;
      this.poseEstimator = new PhotonPoseEstimator(FIELD_LAYOUT, robotToCam);
    }
  }

  public static final String LIMELIGHT_NAME_RIGHT = "limelight-right";
  public static final Matrix<N3, N1> LIMELIGHT_CLOSE_STDDEV_RIGHT =
      VecBuilder.fill(0.01, 0.01, 999999999.9);
  public static final Matrix<N3, N1> LIMELIGHT_FAR_STDDEV_RIGHT =
      VecBuilder.fill(0.05, 0.05, 999999999.9);
  public static final LimelightDetails LIMELIGHT_DETAILS_RIGHT =
      new LimelightDetails(
          LIMELIGHT_NAME_RIGHT, LIMELIGHT_CLOSE_STDDEV_RIGHT, LIMELIGHT_FAR_STDDEV_RIGHT);

  public static final String LIMELIGHT_NAME_RIGHTG = "limelight-rightg";
  public static final Matrix<N3, N1> LIMELIGHT_CLOSE_STDDEV_RIGHTG =
      VecBuilder.fill(0.01, 0.01, 999999999.9);
  public static final Matrix<N3, N1> LIMELIGHT_FAR_STDDEV_RIGHTG =
      VecBuilder.fill(0.05, 0.05, 999999999.9);
  public static final LimelightDetails LIMELIGHT_DETAILS_RIGHTG =
      new LimelightDetails(
          LIMELIGHT_NAME_RIGHTG, LIMELIGHT_CLOSE_STDDEV_RIGHTG, LIMELIGHT_FAR_STDDEV_RIGHTG);

  public static final AprilTagFieldLayout FIELD_LAYOUT =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
  public static final Distance MAX_TAG_DISTANCE = Meters.of(5.0);
  public static final Time LOCALIZATION_PERIOD = Seconds.of(0.02);

  // PhotonVision Cameras
  private static final PhotonCamera camera1 = new PhotonCamera("Camera1");
  public static final Matrix<N3, N1> CAMERA1_CLOSE_STDDEV_RIGHT =
      VecBuilder.fill(0.01, 0.01, 999999999.9);
  public static final Matrix<N3, N1> CAMERA1_FAR_STDDEV_RIGHT =
      VecBuilder.fill(0.05, 0.05, 999999999.9);
  private static final Transform3d camera1RobotToCameraTransform =
      new Transform3d(
          // Meters
          new Translation3d(0, -0.2794, 0.15875),
          new Rotation3d(0.0, Units.degreesToRadians(-44), 90.0));

  public static final PhotonDetails camera1Details =
      new PhotonDetails(
          camera1,
          camera1RobotToCameraTransform,
          CAMERA1_CLOSE_STDDEV_RIGHT,
          CAMERA1_FAR_STDDEV_RIGHT);
}
