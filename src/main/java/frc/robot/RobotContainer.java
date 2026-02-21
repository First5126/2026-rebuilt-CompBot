// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.FMS.Zones;
import frc.robot.constants.AprilTagLocalizationConstants;
import frc.robot.constants.AprilTagLocalizationConstants.PhotonDetails;
import frc.robot.controller.Driver;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandFactory;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ShootingMechanism;
import frc.robot.subsystems.Turret;
import frc.robot.vision.AprilTagLocalization;

public class RobotContainer {
  private double MaxSpeed =
      1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate =
      RotationsPerSecond.of(0.75)
          .in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.1)
          .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
          .withDriveRequestType(
              DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final Telemetry logger = new Telemetry(MaxSpeed);

  public final CommandSwerveDrivetrain m_drivetrain = TunerConstants.createDrivetrain();

  // Declare Subsystems Here

  private Turret m_turret = new Turret();
  private Zones m_zones = new Zones(m_drivetrain::getPose2d);

  private ShootingMechanism m_shootingMechanism =
      new ShootingMechanism(m_turret, m_drivetrain, m_zones);

  // End of Declaring

  PhotonDetails[] photonDetails = {
    // AprilTagLocalizationConstants.camera1Details
  };
  public CommandFactory m_commandFactory =
      new CommandFactory(m_drivetrain, m_turret, m_zones, m_shootingMechanism);

  private AprilTagLocalization m_aprilTagLocalization =
      new AprilTagLocalization(
          m_drivetrain::getPose2d,
          m_drivetrain::resetPose,
          m_drivetrain::addVisionMeasurement,
          m_drivetrain,
          photonDetails,
          AprilTagLocalizationConstants.LIMELIGHT_DETAILS_RIGHT);

  private final SendableChooser<Command> autoChooser;

  /** Creates the container and configures bindings. */
  public RobotContainer() {
    autoChooser = AutoBuilder.buildAutoChooser();
    configureBindings();
  }

  private void configureBindings() {
    Driver.init(m_drivetrain, m_aprilTagLocalization, m_commandFactory, m_zones)
        .configureBindings();

    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    final SwerveRequest idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled()
        .whileTrue(m_drivetrain.applyRequest(() -> idle).ignoringDisable(true));

    m_drivetrain.registerTelemetry(logger::telemeterize);
  }

  /**
   * Builds the autonomous command sequence.
   *
   * @return command to run during autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
