// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.core.CoreCANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FMS.ShiftData;
import java.util.function.DoubleSupplier;

public class LEDLights extends SubsystemBase {

  private static final int kCANdleCANbus = 0;
  private static final CANBus driveBaseCanivore = new CANBus("DriveBase");
  private static CoreCANdle m_candle = new CoreCANdle(kCANdleCANbus, driveBaseCanivore);

  private CANdleConfiguration m_configs = new CANdleConfiguration();

  private final RGBWColor CLEAR = new RGBWColor(255, 255, 255);
  private final RGBWColor RED = new RGBWColor(255, 0, 0);
  private final RGBWColor GREEN = new RGBWColor(0, 255, 0);
  private final RGBWColor BLUE = new RGBWColor(0, 0, 255);
  private final RGBWColor ORANGE = new RGBWColor(255, 157, 0);
  private final RGBWColor PURPLE = new RGBWColor(151, 0, 180);
  private final RainbowAnimation rainbow = new RainbowAnimation(0, 67);
  private SolidColor m_solidColorControl = new SolidColor(0, 67);
  private boolean cleared = false;

  public LEDLights() {
    m_candle.getConfigurator().apply(m_configs);
  }

  public void setRed() {
    m_candle.setControl(m_solidColorControl.withColor(RED));
  }

  public void setGreen() {
    m_candle.setControl(m_solidColorControl.withColor(GREEN));
  }

  public Command setBlue() {
    return run(() -> applyColor(BLUE));
  }

  public Command setPurple() {
    return run(() -> applyColor(PURPLE));
  }

  public Command stopRainbow() {
    return run(() -> m_candle.setControl(m_solidColorControl.withColor(RED)));
  }

  public Command ledByMotion(
      DoubleSupplier rightTrigger,
      DoubleSupplier leftTrigger,
      DoubleSupplier rotationX,
      DoubleSupplier rotationY,
      DoubleSupplier joystickY,
      DoubleSupplier joystickX) {

    return run(
        () -> {
          SmartDashboard.putNumber("Remaning time in Shift", ShiftData.getTimeRemainingInShift());
          SmartDashboard.putNumber(
              "Remaning time in Shift Percentage", ShiftData.getRemainingShiftPercentage());
          boolean moving =
              rightTrigger.getAsDouble() > 0.05
                  || leftTrigger.getAsDouble() > 0.05
                  || rotationX.getAsDouble() > 0.05
                  || rotationY.getAsDouble() > 0.05
                  || joystickX.getAsDouble() > 0.05
                  || joystickY.getAsDouble() > 0.05;

          if (moving) {
            setGreen();
          } else {
            setRed();
          }
        });
  }

  private void applyColor(RGBWColor color) {
    m_candle.setControl(m_solidColorControl.withColor(color));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("RobotCentric", kCANdleCANbus);
  }
}
