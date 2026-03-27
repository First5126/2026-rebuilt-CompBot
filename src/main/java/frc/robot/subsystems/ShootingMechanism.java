package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.FMS.ShiftData;
import frc.robot.FMS.Zones;
import frc.robot.RobotLogger;
import frc.robot.constants.AprilTagLocalizationConstants;
import frc.robot.constants.GoalPoseConstants.GoalPose;
import frc.robot.constants.ShootingMechanismConstants;
import frc.robot.constants.TurretConstants;
import java.util.function.Supplier;

public class ShootingMechanism extends SubsystemBase {
  public static final RobotLogger logger = new RobotLogger("ShootingMechanism");
  private int updateCounter = 1;

  public static class ShootingSolution {
    private Angle predictedHoodAngle;
    private Angle predictedTurretAngle;
    private AngularVelocity predictedFlyWheelVelocity;

    public ShootingSolution(Angle hoodAngle, Angle turretAngle, AngularVelocity flyWheelVelocity) {
      predictedHoodAngle = hoodAngle;
      predictedTurretAngle = turretAngle;
      predictedFlyWheelVelocity = flyWheelVelocity;
    }

    private void set(Angle hoodAngle, Angle turretAngle, AngularVelocity flyWheelVelocity) {
      predictedHoodAngle = hoodAngle;
      predictedTurretAngle = turretAngle;
      predictedFlyWheelVelocity = flyWheelVelocity;
    }

    public Angle getPredictedHoodAngle() {
      return predictedHoodAngle;
    }

    public Angle getPredictedTurretAngle() {
      return predictedTurretAngle;
    }

    public AngularVelocity getPredictedFlyWheelVelocity() {
      return predictedFlyWheelVelocity;
    }
  }

  private static final Angle ZERO_ANGLE = Degrees.of(0);
  private static final AngularVelocity ZERO_ANGULAR_VELOCITY = RotationsPerSecond.of(0);

  private static final double DASHBOARD_UPDATE_PERIOD_SECONDS = 0.1; // 10 Hz
  private static final boolean ENABLE_UPDATE_SHOOTING_SOLUTION_PROFILING = false;

  private final Trigger canShoot;
  private final Turret m_turret;
  private final Hood m_hood;
  private final FlyWheel m_flyWheel;
  private final ShootingSolution m_currentShootingSolution =
      new ShootingSolution(ZERO_ANGLE, ZERO_ANGLE, ZERO_ANGULAR_VELOCITY);
  private final CommandSwerveDrivetrain m_drivetrain;
  private final Zones m_zone;
  private final Field2d m_field = new Field2d();
  private final FieldObject2d m_predictedPoseObject;
  private double m_lastDashboardUpdateSeconds = 0.0;

  public ShootingMechanism(
      Turret turret, CommandSwerveDrivetrain drivetrain, Zones zone, Hood hood, FlyWheel flyWheel) {
    this.m_turret = turret;
    this.m_drivetrain = drivetrain;
    this.m_zone = zone;
    this.m_hood = hood;
    this.m_flyWheel = flyWheel;

    canShoot = new Trigger(this::canShootFuel);
    m_predictedPoseObject = m_field.getObject("Predicted Pose");
    SmartDashboard.putData("Field", m_field);
  }

  public Trigger getCanShoot() {
    return canShoot;
  }

  public ShootingSolution getShootingSolution() {
    return m_currentShootingSolution;
  }

  /**
   * @param robotPoseSupplier The current pose of the robot
   * @param goalPoseSupplier The pose of the goal/target
   * @return A shooting soltuion {@link frc.robot.subsystems.ShootingMechanism.ShootingSolution}
   *     that contains the predicted angle for the hood and turret
   */
  private void updateShootingSolution(
      Supplier<Pose2d> robotPoseSupplier, Supplier<GoalPose> goalPoseSupplier) {

    Pose2d robotPose = robotPoseSupplier.get();
    GoalPose goalPose = goalPoseSupplier.get();

    // check to see if our suppliers are valid
    if (robotPose != null
        && goalPose != null
        && goalPose.pose != null
        && goalPose.interpolationSet != null) {
      Pose2d targetPose = goalPose.pose;

      // find air time from distance
      double distanceToTarget = robotPose.getTranslation().getDistance(targetPose.getTranslation());
      double delayTime =
          goalPose.interpolationSet.distanceToTimeOfFlight.get(distanceToTarget)
              + AprilTagLocalizationConstants.LOCALIZATION_PERIOD.in(Seconds)
              + ShootingMechanismConstants.mechanismDelay.in(Seconds)
              + ShootingMechanismConstants.computationDelay.in(Seconds);

      Pose2d predictedRobotPose = m_drivetrain.getPredictedPose2d(delayTime);

      logger.log("Predicted Robot Pose", predictedRobotPose);

      // get the turret pose
      // Avoid Pose2d allocations in the hot loop.
      double robotHeadingRad = predictedRobotPose.getRotation().getRadians();
      double turretOffsetX = TurretConstants.TURRET_OFFSET.getX();
      double turretOffsetY = TurretConstants.TURRET_OFFSET.getY();
      double cosHeading = Math.cos(robotHeadingRad);
      double sinHeading = Math.sin(robotHeadingRad);
      double turretPoseX =
          predictedRobotPose.getX() + (turretOffsetX * cosHeading) - (turretOffsetY * sinHeading);
      double turretPoseY =
          predictedRobotPose.getY() + (turretOffsetX * sinHeading) + (turretOffsetY * cosHeading);

      // find the distance to target from predicted pose
      double targetDistanceX = targetPose.getX() - turretPoseX;
      double targetDistanceY = targetPose.getY() - turretPoseY;

      double predictedHubDistance = Math.hypot(targetDistanceX, targetDistanceY);

      // find the robot relative angle of the turret (avoid Rotation2d object churn)
      double fieldRelativeAngleRad = Math.atan2(targetDistanceY, targetDistanceX) - Math.PI;
      double predictedTurretAngleRad =
          MathUtil.angleModulus(fieldRelativeAngleRad - robotHeadingRad);
      Angle predictedTurretAngle = Radians.of(predictedTurretAngleRad);

      // find the angle of the hood from the predicted pose
      Angle predictedHoodAngle =
          Degrees.of(goalPose.interpolationSet.distanceToHoodAngle.get(predictedHubDistance));

      // Find the flywheel speed
      AngularVelocity predictedFlyWheelVelocity =
          RotationsPerSecond.of(
              goalPose.interpolationSet.distanceToFlyWheelSpeed.get(predictedHubDistance));

      m_currentShootingSolution.set(
          predictedHoodAngle, predictedTurretAngle, predictedFlyWheelVelocity);

      double nowSeconds = Timer.getFPGATimestamp();
      if (nowSeconds - m_lastDashboardUpdateSeconds >= DASHBOARD_UPDATE_PERIOD_SECONDS) {
        m_lastDashboardUpdateSeconds = nowSeconds;
        logger.log("Valid Shooting Solution", true);

        m_field.setRobotPose(robotPose);
        m_predictedPoseObject.setPose(predictedRobotPose);
      }
    } else {
      double nowSeconds = Timer.getFPGATimestamp();
      if (nowSeconds - m_lastDashboardUpdateSeconds >= DASHBOARD_UPDATE_PERIOD_SECONDS) {
        m_lastDashboardUpdateSeconds = nowSeconds;
        logger.log("Valid Shooting Solution", false);
      }
      m_currentShootingSolution.set(ZERO_ANGLE, ZERO_ANGLE, ZERO_ANGULAR_VELOCITY);
    }
  }

  @Override
  public void periodic() {
    if (ShootingMechanismConstants.updateCounter <= updateCounter) {
      updateShootingSolution(m_drivetrain::getPose2d, m_zone::getGoalPose);
      updateCounter = 1;
    } else {
      updateCounter++;
    }
  }

  private boolean canShootFuel() {
    GoalPose goalPose = m_zone.getGoalPose();
    if (goalPose == null) {
      return false;
    }

    boolean check =
        m_turret
                .getPosition()
                .isNear(
                    m_currentShootingSolution.getPredictedTurretAngle(),
                    ShootingMechanismConstants.turretMaximumError)
            && m_hood
                .getPosition()
                .isNear(
                    m_currentShootingSolution.getPredictedHoodAngle(),
                    ShootingMechanismConstants.hoodMaximumError)
            && m_flyWheel
                .getCurrentSpeed()
                .isNear(
                    m_currentShootingSolution.getPredictedFlyWheelVelocity(),
                    ShootingMechanismConstants.flyWheelMaximumError)
            && (!m_zone.isInDeadZone() || goalPose.requiresShift)
            && (!goalPose.requiresShift || ShiftData.canScore());

    return check;
  }

  public Command startTrackingCommand() {
    // Command trackingCommand =
    // m_turret.rotateToPosition(this::getShootingSolution).alongWith(m_hood.setPosition(this::getShootingSolution));
    Command trackingCommand = m_turret.rotateToPositionAuto(this::getShootingSolution);
    trackingCommand.addRequirements(this);
    return trackingCommand;
  }

  public Command startTrackingCommandAuto() {
    // Command trackingCommand =
    // m_turret.rotateToPosition(this::getShootingSolution).alongWith(m_hood.setPosition(this::getShootingSolution));
    Command trackingCommand = m_turret.rotateToPositionAutoCont(this::getShootingSolution);
    //Command flywheelCommand = m_flyWheel.setSpeedWithSolution(this::getShootingSolution);
    trackingCommand.addRequirements(this);
    return trackingCommand;
  }

  public Command startFlyWheel() {
    Command flywheelCommand = m_flyWheel.setSpeedWithSolution(this::getShootingSolution);
    return flywheelCommand;
  }
}
