// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.FMS;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotLogger;
import frc.robot.constants.GoalPoseConstants;
import frc.robot.constants.GoalPoseConstants.GoalPose;
import frc.robot.constants.ZonesConstants;
import frc.robot.constants.ZonesConstants.Trench;
import frc.robot.constants.ZonesConstants.Zone;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.Optional;
import java.util.function.Supplier;

/** Add your docs here. */
public class Zones {
  private static final double LOG_PERIOD_SECONDS = 0.1;
  private static final double TRENCH_CHECK_PERIOD_SECONDS = 0.1;

  private final Supplier<Pose2d> m_pose;
  private final Supplier<Angle> m_pitch;
  private final Supplier<Angle> m_roll;
  private final Supplier<Optional<Alliance>> m_allianceSupplier;
  private final RobotLogger logger = new RobotLogger("Zones");

  private final CommandSwerveDrivetrain m_commandSwerveDrivetrain;
  private final Pigeon2 m_pigeon;

  private Optional<Alliance> cachedAlliance = Optional.empty();
  private boolean allianceCached = false;
  private double m_lastLogSeconds = 0.0;
  private Alliance m_lastLoggedAlliance = null;
  private double m_lastTrenchCheckSeconds = 0.0;
  private boolean m_cachedNearTrench = false;

  public Zones(CommandSwerveDrivetrain commandSwerveDrivetrain) {
    this(commandSwerveDrivetrain, DriverStation::getAlliance);
  }

  Zones(
      CommandSwerveDrivetrain commandSwerveDrivetrain,
      Supplier<Optional<Alliance>> allianceSupplier) {
    m_commandSwerveDrivetrain = commandSwerveDrivetrain;
    m_allianceSupplier = allianceSupplier;
    m_pigeon = commandSwerveDrivetrain.getPigeon2();

    m_pose = () -> commandSwerveDrivetrain.getPose2d();
    m_pitch = () -> m_pigeon.getPitch().getValue();
    m_roll = () -> m_pigeon.getRoll().getValue();
  }

  private Optional<Alliance> getAlliance() {
    if (!allianceCached) {
      Optional<Alliance> alliance = m_allianceSupplier.get();
      if (alliance.isPresent()) {
        cachedAlliance = alliance;
        allianceCached = true;
      }
    }

    if (cachedAlliance.isPresent() && m_lastLoggedAlliance != cachedAlliance.get()) {
      m_lastLoggedAlliance = cachedAlliance.get();
      logger.log(
          "Alliance", "Retrieved Alliance: " + (cachedAlliance.isPresent() ? cachedAlliance.get().name() : "Unknown"));
    }

    return cachedAlliance;
  }

  public Zone getZone() {
    Translation2d robotTranslation = m_pose.get().getTranslation();
    Zone zone =
        ZonesConstants.firstContainingOrDefault(robotTranslation, Zone.class, Zone.OUT_OF_BOUNDS);

    double nowSeconds = Timer.getFPGATimestamp();
    if (nowSeconds - m_lastLogSeconds >= LOG_PERIOD_SECONDS) {
      m_lastLogSeconds = nowSeconds;
      logger.logAndDisplay("CurrentZone", zone);
    }

    return zone;
  }

  public boolean isNearBump() {
    double pitch = m_pitch.get().in(Degrees);
    double roll = 180 - Math.abs(m_roll.get().in(Degrees));

    boolean onBump =
        Math.abs(pitch) > ZonesConstants.BUMP_ANGLE.in(Degrees)
            || Math.abs(roll) > ZonesConstants.BUMP_ANGLE.in(Degrees);

    return onBump;
  }

  public boolean isNearTrench() {
    double nowSeconds = Timer.getFPGATimestamp();
    if (nowSeconds - m_lastTrenchCheckSeconds >= TRENCH_CHECK_PERIOD_SECONDS) {
      m_lastTrenchCheckSeconds = nowSeconds;
      Pose2d robotPose = m_commandSwerveDrivetrain.getPredictedPose2d(0.25);
      m_cachedNearTrench = ZonesConstants.containsAny(robotPose.getTranslation(), Trench.class);
    }
    return m_cachedNearTrench;
  }

  public boolean isInDeadZone() {
    Translation2d robotTranslation = m_pose.get().getTranslation();
    boolean isInDeadZone = false;

    Optional<Alliance> alliance = getAlliance();
    if (alliance.isPresent()) {
      if (alliance.get() == Alliance.Blue)
        isInDeadZone =
            ZonesConstants.contains(robotTranslation, ZonesConstants.HubDeadZone.BLUE_HUB_DEADZONE);
      else if (alliance.get() == Alliance.Red)
        isInDeadZone =
            ZonesConstants.contains(robotTranslation, ZonesConstants.HubDeadZone.RED_HUB_DEADZONE);
    }
    logger.log("IsInDeadZone", isInDeadZone);
    return isInDeadZone;
  }

  public Pose2d getTurretShootingPose() {
    return getGoalPose().pose;
  }

  public GoalPose getGoalPose() {
    Zone zone = getZone();
    Optional<Alliance> alliance = getAlliance();
    if (!alliance.isPresent()) {
      logger.log("GoalPose", "Alliance information not available, defaulting to Blue Hub pose");
      return GoalPoseConstants.BLUE_HUB;
    }

    switch (zone) {
      case BLUE_ZONE:
        return alliance.get() == Alliance.Blue
            ? GoalPoseConstants.BLUE_HUB
            : GoalPoseConstants.RED_HUB;
      case NEUTRAL_ZONE_LEFT:
        return alliance.get() == Alliance.Blue
            ? GoalPoseConstants.BLUE_LEFT_SIDE
            : GoalPoseConstants.RED_LEFT_SIDE;
      case NEUTRAL_ZONE_RIGHT:
        return alliance.get() == Alliance.Blue
            ? GoalPoseConstants.BLUE_RIGHT_SIDE
            : GoalPoseConstants.RED_RIGHT_SIDE;
      case RED_ZONE:
        return alliance.get() == Alliance.Blue
            ? GoalPoseConstants.BLUE_HUB
            : GoalPoseConstants.RED_HUB;
      default:
        break;
    }
    return alliance.get() == Alliance.Blue ? GoalPoseConstants.BLUE_HUB : GoalPoseConstants.RED_HUB;
  }
}
