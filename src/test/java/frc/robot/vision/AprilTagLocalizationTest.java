package frc.robot.vision;

import static edu.wpi.first.units.Units.Meters;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.constants.AprilTagLocalizationConstants.LimelightDetails;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

class AprilTagLocalizationTest {
  @AfterEach
  void tearDown() {
    LimelightHelpers.clear();
  }

  @Test
  void poseEstimate_fullTrust_resetsPoseWithSupplierYaw() {
    var limelightName = "limelight-test";
    LimelightDetails limelightDetails =
        new LimelightDetails(
            limelightName, VecBuilder.fill(1.0, 2.0, 3.0), VecBuilder.fill(4.0, 5.0, 6.0));

    var suppliedPose = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(90.0));
    var poseSupplier = (java.util.function.Supplier<Pose2d>) () -> suppliedPose;

    AtomicReference<Pose2d> resetPoseValue = new AtomicReference<>();
    AprilTagLocalization.ResetPose resetPose = resetPoseValue::set;

    AtomicInteger visionCalls = new AtomicInteger(0);
    AprilTagLocalization.VisionConsumer visionConsumer =
        (pose, ts, stdDevs) -> visionCalls.incrementAndGet();

    var localization =
        new AprilTagLocalization(
            poseSupplier,
            resetPose,
            visionConsumer,
            null,
            null,
            new frc.robot.constants.AprilTagLocalizationConstants.PhotonDetails[] {},
            limelightDetails);
    localization.setFullTrust(true);

    LimelightHelpers.setPoseEstimate(
        limelightName,
        new LimelightHelpers.PoseEstimate(
            new Pose2d(1.0, 2.0, Rotation2d.fromDegrees(15.0)), 123.0, 99.0));

    localization.poseEstimate();

    assertEquals(0, visionCalls.get());
    Pose2d resetPoseResult = resetPoseValue.get();
    assertNotNull(resetPoseResult);
    assertEquals(1.0, resetPoseResult.getX(), 1e-9);
    assertEquals(2.0, resetPoseResult.getY(), 1e-9);
    assertEquals(90.0, resetPoseResult.getRotation().getDegrees(), 1e-9);
  }

  @Test
  void poseEstimate_nonTrust_rejectsOffFieldPose() {
    var limelightName = "limelight-test";
    LimelightDetails limelightDetails =
        new LimelightDetails(
            limelightName, VecBuilder.fill(1.0, 1.0, 1.0), VecBuilder.fill(2.0, 2.0, 2.0));

    var poseSupplier = (java.util.function.Supplier<Pose2d>) Pose2d::new;

    AtomicInteger resetCalls = new AtomicInteger(0);
    AprilTagLocalization.ResetPose resetPose = pose -> resetCalls.incrementAndGet();

    AtomicInteger visionCalls = new AtomicInteger(0);
    AprilTagLocalization.VisionConsumer visionConsumer =
        (pose, ts, stdDevs) -> visionCalls.incrementAndGet();

    var localization =
        new AprilTagLocalization(
            poseSupplier,
            resetPose,
            visionConsumer,
            null,
            null,
            new frc.robot.constants.AprilTagLocalizationConstants.PhotonDetails[] {},
            limelightDetails);
    localization.setFullTrust(false);

    LimelightHelpers.setPoseEstimate(
        limelightName,
        new LimelightHelpers.PoseEstimate(new Pose2d(-0.1, 1.0, new Rotation2d()), 1.0, 1.0));

    localization.poseEstimate();

    assertEquals(0, resetCalls.get());
    assertEquals(0, visionCalls.get());
  }

  @Test
  void poseEstimate_nonTrust_rejectsTooFarPose() {
    var limelightName = "limelight-test";
    LimelightDetails limelightDetails =
        new LimelightDetails(
            limelightName, VecBuilder.fill(1.0, 1.0, 1.0), VecBuilder.fill(2.0, 2.0, 2.0));

    var poseSupplier = (java.util.function.Supplier<Pose2d>) Pose2d::new;

    AtomicInteger resetCalls = new AtomicInteger(0);
    AprilTagLocalization.ResetPose resetPose = pose -> resetCalls.incrementAndGet();

    AtomicInteger visionCalls = new AtomicInteger(0);
    AprilTagLocalization.VisionConsumer visionConsumer =
        (pose, ts, stdDevs) -> visionCalls.incrementAndGet();

    var localization =
        new AprilTagLocalization(
            poseSupplier,
            resetPose,
            visionConsumer,
            null,
            null,
            new frc.robot.constants.AprilTagLocalizationConstants.PhotonDetails[] {},
            limelightDetails);
    localization.setFullTrust(false);

    double tooFarMeters = 6.0;
    LimelightHelpers.setPoseEstimate(
        limelightName,
        new LimelightHelpers.PoseEstimate(
            new Pose2d(1.0, 1.0, new Rotation2d()), 1.0, tooFarMeters));

    localization.poseEstimate();

    assertEquals(0, resetCalls.get());
    assertEquals(0, visionCalls.get());
  }

  @Test
  void poseEstimate_nonTrust_acceptsAndInterpolatesStdDevs() {
    var limelightName = "limelight-test";
    Matrix<N3, N1> close = VecBuilder.fill(1.0, 2.0, 3.0);
    Matrix<N3, N1> far = VecBuilder.fill(5.0, 6.0, 7.0);
    LimelightDetails limelightDetails = new LimelightDetails(limelightName, close, far);

    var poseSupplier = (java.util.function.Supplier<Pose2d>) Pose2d::new;
    AprilTagLocalization.ResetPose resetPose = pose -> {};

    AtomicReference<Pose2d> visionPose = new AtomicReference<>();
    AtomicReference<Double> visionTimestamp = new AtomicReference<>();
    AtomicReference<Matrix<N3, N1>> visionStdDevs = new AtomicReference<>();
    AprilTagLocalization.VisionConsumer visionConsumer =
        (pose, ts, stdDevs) -> {
          visionPose.set(pose);
          visionTimestamp.set(ts);
          visionStdDevs.set(stdDevs);
        };

    var localization =
        new AprilTagLocalization(
            poseSupplier,
            resetPose,
            visionConsumer,
            null,
            null,
            new frc.robot.constants.AprilTagLocalizationConstants.PhotonDetails[] {},
            limelightDetails);
    localization.setFullTrust(false);

    double halfMaxDistance = 2.5;
    assertEquals(
        5.0, frc.robot.constants.AprilTagLocalizationConstants.MAX_TAG_DISTANCE.in(Meters));

    LimelightHelpers.setPoseEstimate(
        limelightName,
        new LimelightHelpers.PoseEstimate(
            new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(30.0)), 42.0, halfMaxDistance));

    localization.poseEstimate();

    assertNotNull(LimelightHelpers.lastOrientationCall);
    assertEquals(limelightName, LimelightHelpers.lastOrientationCall.limelightName());
    assertEquals(0.0, LimelightHelpers.lastOrientationCall.yaw(), 1e-9);
    assertEquals(0.0, LimelightHelpers.lastOrientationCall.yawRate(), 1e-9);
    assertNotNull(visionPose.get());
    assertEquals(1.0, visionPose.get().getX(), 1e-9);
    assertEquals(1.0, visionPose.get().getY(), 1e-9);
    assertEquals(42.0, visionTimestamp.get(), 1e-9);

    Matrix<N3, N1> stdDevs = visionStdDevs.get();
    assertNotNull(stdDevs);
    assertEquals(3.0, stdDevs.get(0, 0), 1e-9);
    assertEquals(4.0, stdDevs.get(1, 0), 1e-9);
    assertEquals(5.0, stdDevs.get(2, 0), 1e-9);
  }
}
