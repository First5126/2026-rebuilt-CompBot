package frc.robot.controller;

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
