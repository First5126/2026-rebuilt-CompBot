package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.FMS.ShiftData;
import frc.robot.FMS.Zones;
import frc.robot.constants.GoalPoseConstants.GoalPose;
import frc.robot.constants.FlyWheelConstants;
import frc.robot.constants.HoodConstants;
import frc.robot.constants.ShootingMechanismConstants;
import frc.robot.constants.TurretConstants;
import java.util.function.Supplier;

public class ShootingMechanism extends SubsystemBase {
  public static class ShootingSolution {
    public Angle predictedHoodAngle;
    public Angle predictedTurretAngle;
    public AngularVelocity predictedFlyWheelVelocity;

    public ShootingSolution(Angle hoodAngle, Angle turretAngle, AngularVelocity flyWheelVelocity) {
      predictedHoodAngle = hoodAngle;
      predictedTurretAngle = turretAngle;
      predictedFlyWheelVelocity = flyWheelVelocity;
    }
  }

  private final Trigger canShoot;
  private Turret m_turret;
  private Hood m_hood;
  private FlyWheel m_flyWheel;
  private ShootingSolution m_currentShootingSolution =
      new ShootingSolution(Degrees.of(0), Degrees.of(0), RotationsPerSecond.of(0));
  private CommandSwerveDrivetrain m_drivetrain;
  private Zones m_zone;

  public ShootingMechanism(
      Turret m_turret, CommandSwerveDrivetrain m_drivetrain, Zones m_zone, Hood m_hood, FlyWheel m_flyWheel) {
    this.m_turret = m_turret;
    this.m_drivetrain = m_drivetrain;
    this.m_zone = m_zone;
    this.m_hood = m_hood;
    this.m_flyWheel = m_flyWheel;

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
  private void updateShootingSolution(
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

      double predicatedHubDistance = Math.hypot(targetDistanceX, targetDistanceY);

      // find the field relative angle from the distance
      Rotation2d fieldRelativeAngle =
          Rotation2d.fromRadians(Math.atan2(targetDistanceY, targetDistanceX) - Math.PI);

      // find the robot realtive angle of the turret
      m_currentShootingSolution.predictedTurretAngle =
          fieldRelativeAngle
              .minus(
                  robotPose
                      .getRotation()
                      .plus(new Rotation2d(robotSpeeds.omegaRadiansPerSecond * 0.02)))
              .getMeasure();

      // find the angle of the hood from the predicted pose
      m_currentShootingSolution.predictedHoodAngle =
          Degrees.of(
              HoodConstants.DISTANCE_TO_ANGLE_INTERPOLATOR.get(
                  predicatedHubDistance));

      // Find the flywheel speed
      m_currentShootingSolution.predictedFlyWheelVelocity = RotationsPerSecond.of(FlyWheelConstants.DISTANCE_TO_SPEED_INTERPOLATOR
      .get(predicatedHubDistance));

      SmartDashboard.putNumber("Hood Angle Interpolated (Deg)", m_currentShootingSolution.predictedHoodAngle.in(Degrees));
      SmartDashboard.putNumber("Turrent Angle Calculated (Deg)", m_currentShootingSolution.predictedTurretAngle.in(Degrees));
      SmartDashboard.putNumber("FlyWheel Interpolated (RPS)", m_currentShootingSolution.predictedFlyWheelVelocity.in(RotationsPerSecond));
    } else {
      SmartDashboard.putBoolean("Valid Shooting Solution", false);
    }
  }

  @Override
  public void periodic() {
        updateShootingSolution(
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
                    ShootingMechanismConstants.turretMaximumError) &&
        m_hood.getPosition()
              .isNear(m_currentShootingSolution.predictedHoodAngle, 
              ShootingMechanismConstants.hoodMaximumError) &&
        m_flyWheel.getCurrentSpeed()
              .isNear(m_currentShootingSolution.predictedFlyWheelVelocity, 
              ShootingMechanismConstants.flyWheelMaximumError)
            && (!goalPose.requiresShift || ShiftData.canScore());

    // TODO: add hood
    SmartDashboard.putNumber(
        "Turret Deviation Deg",
        m_turret.getPosition().minus(m_currentShootingSolution.predictedTurretAngle).in(Degrees));
    return check;
  }

  public Command startTrackingCommand() {
    // Command trackingCommand =
    // m_turret.rotateToPosition(this::getShootingSolution).alongWith(m_hood.setPosition(this::getShootingSolution));
    Command trackingCommand = m_turret.rotateToPosition(this::getShootingSolution);
    trackingCommand.addRequirements(this);
    return trackingCommand;
  }
}
