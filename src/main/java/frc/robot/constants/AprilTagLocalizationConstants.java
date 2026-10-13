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
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
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
      VecBuilder.fill(0.03, 0.03, 999999999.9);
  public static final Matrix<N3, N1> LIMELIGHT_FAR_STDDEV_RIGHT =
      VecBuilder.fill(0.20, 0.20, 999999999.9);
  public static final LimelightDetails LIMELIGHT_DETAILS_RIGHT =
      new LimelightDetails(
          LIMELIGHT_NAME_RIGHT, LIMELIGHT_CLOSE_STDDEV_RIGHT, LIMELIGHT_FAR_STDDEV_RIGHT);

  public static final String LIMELIGHT_NAME_LEFT = "limelight-left";
  public static final Matrix<N3, N1> LIMELIGHT_CLOSE_STDDEV_LEFT =
      VecBuilder.fill(0.03, 0.03, 999999999.9);
  public static final Matrix<N3, N1> LIMELIGHT_FAR_STDDEV_LEFT =
      VecBuilder.fill(0.20, 0.20, 999999999.9);
  public static final LimelightDetails LIMELIGHT_DETAILS_LEFT =
      new LimelightDetails(
          LIMELIGHT_NAME_LEFT, LIMELIGHT_CLOSE_STDDEV_LEFT, LIMELIGHT_FAR_STDDEV_LEFT);

  public static final AprilTagFieldLayout FIELD_LAYOUT =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
  public static final Distance MAX_TAG_DISTANCE = Meters.of(5.0);
  public static final Time LOCALIZATION_PERIOD = Seconds.of(0.02);

  public static final int MIN_TAG_COUNT = 2;
  public static final Distance MIN_TAG_DISTANCE = Meters.of(0.25);
  public static final Distance MAX_MEASUREMENT_JUMP = Meters.of(2.0);
}
