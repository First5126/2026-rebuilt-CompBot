package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.FMS.ShiftData;
import frc.robot.FMS.Zones;
import frc.robot.constants.GoalPoseConstants.GoalPose;
import frc.robot.constants.HoodConstants;
import frc.robot.constants.ShootingMechanismConstants;
import frc.robot.constants.TurretConstants;
import java.util.function.Supplier;

public class ShootingMechanism extends SubsystemBase {
  public static class ShootingSolution {
    public Angle predictedHoodAngle;
    public Angle predictedTurretAngle;

    public ShootingSolution(Angle hoodAngle, Angle turretAngle) {
      predictedHoodAngle = hoodAngle;
      predictedTurretAngle = turretAngle;
    }
  }

  private final Trigger canShoot;
  private Turret m_turret;
  private Hood m_hood;
  private ShootingSolution m_currentShootingSolution =
      new ShootingSolution(Degrees.of(0), Degrees.of(0));
  private CommandSwerveDrivetrain m_drivetrain;
  private Zones m_zone;

  public ShootingMechanism(Turret m_turret, CommandSwerveDrivetrain m_drivetrain, Zones m_zone) {
    this.m_turret = m_turret;
    this.m_drivetrain = m_drivetrain;
    this.m_zone = m_zone;
    // TODO: get the hood too

    canShoot = new Trigger(this::canShootFuel);
  }

  public Trigger getCanShoot() {
    return canShoot;
  }

  public ShootingSolution getShootingSolution() {
    return m_currentShootingSolution;
  }

  /**
   * @param robotPoseSupplier The current pose of the robot
   * @param speed The chassis speeds of the drivetrain
   * @param targetPoseSupplier The pose of the target
   * @return A shooting soltuion {@link frc.robot.subsystems.ShootingMechanism.ShootingSolution}
   *     that contains the predicted angle for the hood and turret
   */
  private ShootingSolution getShootingSolution(
      Supplier<Pose2d> robotPoseSupplier,
      Supplier<ChassisSpeeds> speed,
      Supplier<Pose2d> targetPoseSupplier) {

    // check to see if our suppliers are valid
    if (robotPoseSupplier.get() != null
        && targetPoseSupplier.get() != null
        && speed.get() != null) {
      SmartDashboard.putBoolean("Valid Shooting Solution", true);

      // retreive the value of all the suppliers
      Pose2d robotPose = robotPoseSupplier.get();
      ChassisSpeeds robotSpeeds = speed.get();
      Pose2d targetPose = targetPoseSupplier.get();

      // find air time from distance
      double distanceToTarget = robotPose.getTranslation().getDistance(targetPose.getTranslation());
      double airTime = TurretConstants.DISTANCE_TO_TIME_INTERPOLATOR.get(distanceToTarget);

      // find how far we travel by the time the ball will reach the target
      double predicatedDistance =
          airTime * Math.hypot(robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond);

      // find the angle of the the speeds that are currently in robotcentric
      Rotation2d rotation =
          new Rotation2d(Math.atan2(robotSpeeds.vyMetersPerSecond, robotSpeeds.vxMetersPerSecond));

      // add the angle of the speeds to get the field centric velocity angle
      rotation = rotation.plus(robotPose.getRotation());

      // find the predicated x and y of our robot pose
      double predictedX = robotPose.getX() + predicatedDistance * Math.cos(rotation.getRadians());
      double predictedY = robotPose.getY() + predicatedDistance * Math.sin(rotation.getRadians());

      // get the turret pose
      Pose2d turretPose =
          new Pose2d(predictedX, predictedY, new Rotation2d()).plus(TurretConstants.TURRET_OFFSET);

      // find the distance to target from predicted pose
      double targetDistanceX = targetPose.getX() - turretPose.getX();
      double targetDistanceY = targetPose.getY() - turretPose.getY();

      // find the field relative angle from the distance
      Rotation2d fieldRelativeAngle =
          Rotation2d.fromRadians(Math.atan2(targetDistanceY, targetDistanceX));

      // find the robot realtive angle of the turret
      Angle robotRelativeAngle =
          fieldRelativeAngle
              .minus(
                  robotPose
                      .getRotation()
                      .plus(new Rotation2d(robotSpeeds.omegaRadiansPerSecond * 0.02)))
              .getMeasure();

      // find the angle of the hood from the predicted pose
      Angle hoodAngle =
          Rotations.of(
              HoodConstants.DISTANCE_TO_ANGLE_INTERPOLATOR.get(
                  Math.hypot(targetDistanceX, targetDistanceY)));

      return new ShootingSolution(hoodAngle, robotRelativeAngle);
    } else {
      SmartDashboard.putBoolean("Valid Shooting Solution", false);
      return new ShootingSolution(Degrees.of(0), Degrees.of(0));
    }
  }

  @Override
  public void periodic() {
    m_currentShootingSolution =
        getShootingSolution(
            m_drivetrain::getPose2d, m_drivetrain::getSpeeds, m_zone::getTurretShootingPose);

    SmartDashboard.putBoolean("Can Shoot", canShoot.getAsBoolean());
  }

  private boolean canShootFuel() {

    GoalPose goalPose = m_zone.getGoalPose();

    boolean check =
        m_turret
                .getPosition()
                .isNear(
                    m_currentShootingSolution.predictedTurretAngle,
                    ShootingMechanismConstants.turretMaximumError)
            && (!goalPose.requiresShift || ShiftData.canScore() == true);

    // TODO: add hood
    SmartDashboard.putNumber(
        "Turret Deviation Deg",
        m_turret.getPosition().minus(m_currentShootingSolution.predictedTurretAngle).in(Degrees));
    return check;
  }
}
