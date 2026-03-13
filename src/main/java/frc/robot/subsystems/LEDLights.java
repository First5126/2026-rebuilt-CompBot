// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.LarsonAnimation;
import com.ctre.phoenix6.hardware.core.CoreCANdle;
import com.ctre.phoenix6.signals.LarsonBounceValue;
import com.ctre.phoenix6.signals.RGBWColor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FMS.ShiftData;
import frc.robot.FMS.Zones;
import frc.robot.constants.CANConstants;
import frc.robot.constants.LEDConstants;
import java.util.function.Supplier;

public class LEDLights extends SubsystemBase {

  private static CoreCANdle m_candle = new CoreCANdle(CANConstants.candle, CANConstants.mechanismCanivore);

  private CANdleConfiguration m_configs = new CANdleConfiguration();

  public LEDLights(ShiftData shiftData) {
    LEDConstants.setShiftData(shiftData);
    m_candle.getConfigurator().apply(m_configs);
  }

  public Command ledOnZoneCommand(Zones zone) {
    return run(
        () -> {
          switch (zone.getZone()) {
            case ALLIANCE_ZONE:
              applyColor(LEDConstants.BLUE);
              break;
            case NEUTRAL_ZONE_LEFT:
            case NEUTRAL_ZONE_RIGHT:
              applyColor(LEDConstants.PURPLE);
              break;
            case OPPONENT_ZONE:
              applyColor(LEDConstants.RED);
              break;
            default:
              applyColor(LEDConstants.GREEN);
              break;
          }
        });
  }

  public Command canScore() {
    boolean canScore = ShiftData.canScore();
    return run(
        () -> {
          if (canScore == true) {
            applyColor(LEDConstants.GREEN);
          } else if (canScore == false) {
            applyColor(LEDConstants.RED);
          }
        });
  }

  public Command ledByShifts() {
    return run(
        () -> {
          Supplier<Double> timeLeft = () -> ShiftData.getTimeRemainingInShift();
          RGBWColor color = LEDConstants.PURPLE;
          if (ShiftData.canScore() == true) {
            color = LEDConstants.GREEN;
          } else {
            color = LEDConstants.RED;
          }
          SmartDashboard.putNumber("timeLeft", timeLeft.get());
          SmartDashboard.putNumber("Shift duration", ShiftData.getShift().getDuration());

          // must initialize the leds on when starting the stage
          // start removing colors from the led
          int ledCount = (int) Math.round(timeLeft.get());
          int ammountLightingUp =
              LEDConstants.COUNTDOWN_LED_END_INDEX - LEDConstants.COUNTDOWN_LED_START_INDEX;
          SmartDashboard.putNumber("Ammount of leds", ledCount);
          int endIndex =
              (int)
                  (LEDConstants.COUNTDOWN_LED_END_INDEX
                      - Math.ceil((double) (ammountLightingUp - ledCount) / 2));
          int startIndex =
              (int)
                  (LEDConstants.COUNTDOWN_LED_START_INDEX
                      + Math.floor((double) (ammountLightingUp - ledCount) / 2));
          applyColorWithIndex(
              LEDConstants.BLACK, LEDConstants.COUNTDOWN_LED_START_INDEX, startIndex);
          applyColorWithIndex(LEDConstants.BLACK, endIndex, LEDConstants.COUNTDOWN_LED_END_INDEX);
          if ((int) Math.round(timeLeft.get()) == (int) ShiftData.getShift().getDuration()) {
            applyColorWithIndex(color, startIndex, endIndex);
            // System.out.println("The same timeleft and duration");
          }
        });
  }

  public Command knightRiderCommand() {
    return run(
        () -> {
          m_candle.setControl(LEDConstants.knightRiderAnimation);
        });
  }

  private void applyColor(RGBWColor color) {
    m_candle.setControl(LEDConstants.m_solidColorControl.withColor(color));
  }

  private void applyColorWithIndex(RGBWColor color, int start, int end) {
    m_candle.setControl(
        LEDConstants.m_solidColorControl
            .withColor(color)
            .withLEDStartIndex(start)
            .withLEDEndIndex(end));
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Can Score", ShiftData.canScore());
  }
}