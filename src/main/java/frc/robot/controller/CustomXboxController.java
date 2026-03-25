package frc.robot.controller;

// import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.ControllerConstants;

public class CustomXboxController extends CommandXboxController {
  public static double modifyAxisWithCustomDeadband(double value, double deadband, int power) {
    value = MathUtil.applyDeadband(value, deadband);
    value = Math.copySign(Math.pow(value, power), value);
    return value;
  }

  public static double modifyAxis(double value) {
    return modifyAxis(value, 1);
  }

  public static double modifyAxis(double value, int exponent) {
    // Deadband
    value = MathUtil.applyDeadband(value, ControllerConstants.DEADBAND);
    value = Math.copySign(Math.pow(value, exponent), value);
    return value;
  }

  public CustomXboxController(int port) {
    super(port);
  }

  /*public void logDogLog(final String prefix) {
    XboxController hid = getHID();

    DogLog.log(prefix + "/Connected", hid.isConnected());
    DogLog.log(prefix + "/POV", hid.getPOV());

    DogLog.log(prefix + "/Axes/LeftXRaw", hid.getLeftX());
    DogLog.log(prefix + "/Axes/LeftYRaw", hid.getLeftY());
    DogLog.log(prefix + "/Axes/RightXRaw", hid.getRightX());
    DogLog.log(prefix + "/Axes/RightYRaw", hid.getRightY());
    DogLog.log(prefix + "/Axes/LeftTriggerRaw", hid.getLeftTriggerAxis());
    DogLog.log(prefix + "/Axes/RightTriggerRaw", hid.getRightTriggerAxis());

    DogLog.log(prefix + "/Axes/LeftX", getLeftX());
    DogLog.log(prefix + "/Axes/LeftY", getLeftY());
    DogLog.log(prefix + "/Axes/RightX", getRightX());
    DogLog.log(prefix + "/Axes/RightY", getRightY());
    DogLog.log(prefix + "/Axes/LeftTrigger", getLeftTriggerAxis());
    DogLog.log(prefix + "/Axes/RightTrigger", getRightTriggerAxis());

    DogLog.log(prefix + "/Buttons/A", hid.getAButton());
    DogLog.log(prefix + "/Buttons/B", hid.getBButton());
    DogLog.log(prefix + "/Buttons/X", hid.getXButton());
    DogLog.log(prefix + "/Buttons/Y", hid.getYButton());
    DogLog.log(prefix + "/Buttons/Back", hid.getBackButton());
    DogLog.log(prefix + "/Buttons/Start", hid.getStartButton());
    DogLog.log(prefix + "/Buttons/LeftBumper", hid.getLeftBumperButton());
    DogLog.log(prefix + "/Buttons/RightBumper", hid.getRightBumperButton());
    DogLog.log(prefix + "/Buttons/LeftStick", hid.getLeftStickButton());
    DogLog.log(prefix + "/Buttons/RightStick", hid.getRightStickButton());
  }*/

  @Override
  public double getLeftX() {
    return getAxisWithDeadband(super.getLeftX());
  }

  @Override
  public double getLeftY() {
    return getAxisWithDeadband(super.getLeftY());
  }

  @Override
  public double getRightX() {
    return getAxisWithDeadband(super.getRightX());
  }

  @Override
  public double getRightY() {
    return getAxisWithDeadband(super.getRightY());
  }

  // isolate the logic for getting the deadband
  private double getAxisWithDeadband(final double value) {
    return MathUtil.applyDeadband(value, ControllerConstants.DEADBAND);
  }
}
