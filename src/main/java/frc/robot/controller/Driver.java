package frc.robot.controller;

import frc.robot.FMS.Zones;
import frc.robot.constants.ControllerConstants;
import frc.robot.subsystems.CommandFactory;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import lombok.Getter;
import lombok.Setter;

public class Driver extends CustomXboxController implements Controller {
  // Singleton instance
  private static Driver INSTANCE;

  @Getter @Setter private CommandSwerveDrivetrain drivetrain;
  @Getter @Setter private CommandFactory commandFactory;
  @Getter @Setter private Zones zone;

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
      CommandSwerveDrivetrain drivetrain, CommandFactory commandFactory, Zones zone) {
    Driver driver = getInstance();
    driver.setDrivetrain(drivetrain);
    driver.setCommandFactory(commandFactory);
    driver.setZone(zone);

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

    // Reset the field-centric heading on left bumper press.
    this.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

    return this;
  }
}
