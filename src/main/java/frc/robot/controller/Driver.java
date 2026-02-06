package frc.robot.controller;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.ControllerConstants;
import frc.robot.subsystems.CommandFactory;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
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
  @Getter @Setter private Intake intake = new Intake();
  @Getter @Setter private Turret turret = new Turret();

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

  public static Driver init(CommandSwerveDrivetrain drivetrain,
      AprilTagLocalization aprilTagLocalization,
      CommandFactory commandFactory) {
    Driver driver = getInstance();
    driver.setDrivetrain(drivetrain);
    driver.setAprilTagLocalization(aprilTagLocalization);
    driver.setCommandFactory(commandFactory);
    driver.setTurret(new Turret());
    driver.setIntake(new Intake());

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
            this::getLeftX));


    this.a().onTrue(aprilTagLocalization.setTrust(true));
    this.a().onFalse(aprilTagLocalization.setTrust(false));

    // Reset the field-centric heading on left bumper press.
    SmartDashboard.putNumber("Pose X", 0);
    SmartDashboard.putNumber("Pose Y", 0);

    // Reset the field-centric heading on left bumper press.
    this.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

    return this;
  }
}
