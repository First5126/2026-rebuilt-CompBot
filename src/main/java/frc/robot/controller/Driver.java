package frc.robot.controller;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.swerve.SwerveRequest;
import frc.robot.FMS.Zones;
import frc.robot.constants.ControllerConstants;
import frc.robot.subsystems.CommandFactory;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.FlyWheel;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeDeployer;
import frc.robot.subsystems.ShootingMechanism;
import frc.robot.subsystems.Turret;
import frc.robot.vision.AprilTagLocalization;
import lombok.Getter;
import lombok.Setter;

public class Driver extends CustomXboxController implements Controller {
  // Singleton instance
  private static Driver INSTANCE;

  @Getter @Setter private CommandSwerveDrivetrain drivetrain;
  @Getter @Setter private AprilTagLocalization aprilTagLocalization;
  @Getter @Setter private CommandFactory commandFactory;
  @Getter @Setter private IntakeDeployer intake;
  @Getter @Setter private Turret turret;
  @Getter @Setter private Zones zone;
  @Getter @Setter private Indexer indexer;
  @Getter @Setter private FlyWheel flyWheel;
  @Getter @Setter private ShootingMechanism shootingMechanism;
  @Getter @Setter private Hood hood;

  private final SwerveRequest.SwerveDriveBrake BRAKE = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt POINT = new SwerveRequest.PointWheelsAt();

  // Private constructor to prevent instantiation from outside
  private Driver() {
    super(ControllerConstants.DRIVER_CONTROLLER_PORT);
  }

  // Public method to access the single instance
  public static Driver getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new Driver();
    }
    return INSTANCE;
  }

  public static Driver init(
      CommandSwerveDrivetrain drivetrain,
      AprilTagLocalization aprilTagLocalization,
      CommandFactory commandFactory,
      IntakeDeployer intake,
      Turret turret,
      Zones zone,
      Indexer indexer,
      FlyWheel flyWheel,
      Hood hood,
      ShootingMechanism shootingMechanism) {
    Driver driver = getInstance();
    driver.setDrivetrain(drivetrain);
    driver.setAprilTagLocalization(aprilTagLocalization);
    driver.setCommandFactory(commandFactory);
    driver.setIntake(intake);
    driver.setTurret(turret);
    driver.setZone(zone);
    driver.setIndexer(indexer);
    driver.setFlyWheel(flyWheel);
    driver.setHood(hood);
    driver.setShootingMechanism(shootingMechanism);

    return driver;
  }

  @Override
  public Driver configureBindings() {

    drivetrain.setDefaultCommand(
        drivetrain.gasPedalCommand(
            this::getRightTriggerAxis,
            this::getLeftTriggerAxis,
            this::getRightX,
            this::getLeftY,
            this::getLeftX,
            zone));

    // this.a().onTrue(aprilTagLocalization.setTrust(true));
    // this.a().onFalse(aprilTagLocalization.setTrust(false));

    this.povUp().onTrue(hood.setVoltage(Volts.of(0.5))).onFalse(hood.setVoltage(Volts.of(0)));
    this.povDown().onTrue(hood.setVoltage(Volts.of(-0.5))).onFalse(hood.setVoltage(Volts.of(0)));

    this.a().onTrue(indexer.startIndexing()).onFalse(indexer.stopIndexing());

    // Reset the field-centric heading on left bumper press.
    this.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
    // this.a().onTrue(indexer.startIndexing()).onFalse(indexer.stopIndexing());
    this.b()
        .whileTrue(flyWheel.startShootingWithInterpolation(drivetrain::getPose2d,zone::getTurretShootingPose).alongWith(hood.setPosition(shootingMechanism::getShootingSolution)))
        .onFalse(flyWheel.stopSpinning());

    this.y().whileTrue(flyWheel.setSpeed(flyWheel::getDashboardSpeedRPS).alongWith(hood.setPositionToDashboard()))
    .onFalse(flyWheel.stopSpinning());

    this.x().whileTrue(commandFactory.startShooting()).onFalse(commandFactory.stopShooting());

    return this;
  }
}
