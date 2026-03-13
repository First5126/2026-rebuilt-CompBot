// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import com.ctre.phoenix6.controls.LarsonAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.signals.LarsonBounceValue;
import com.ctre.phoenix6.signals.RGBWColor;
import frc.robot.FMS.ShiftData;

public class LEDConstants {
  public static final RGBWColor CLEAR = new RGBWColor(255, 255, 255);
  public static final RGBWColor RED = new RGBWColor(255, 0, 0);
  public static final RGBWColor GREEN = new RGBWColor(0, 255, 0);
  public static final RGBWColor BLUE = new RGBWColor(0, 0, 255);
  public static final RGBWColor ORANGE = new RGBWColor(255, 157, 0);
  public static final RGBWColor PURPLE = new RGBWColor(151, 0, 180);
  public static final RGBWColor BLACK = new RGBWColor(0, 0, 0);
  public static final RGBWColor KNIGHT_RIDER = new RGBWColor(128, 0, 0);
  public static final int END_INDEX = 67;
  public static final int START_INDEX = 0;
  public static final int COUNTDOWN_LED_START_INDEX = 0;
  public static final int COUNTDOWN_LED_END_INDEX = 26;
  public static final RainbowAnimation rainbow = new RainbowAnimation(START_INDEX, END_INDEX);
  public static final SolidColor m_solidColorControl = new SolidColor(START_INDEX, END_INDEX);
  private static ShiftData m_shiftData;

  public static final LarsonAnimation knightRiderAnimation =
      new LarsonAnimation(LEDConstants.START_INDEX, LEDConstants.END_INDEX)
          .withBounceMode(LarsonBounceValue.Front)
          .withSize(8)
          .withColor(LEDConstants.KNIGHT_RIDER);

  public static ShiftData getShiftData() {
    return m_shiftData;
  }

  public static void setShiftData(ShiftData shiftData) {
    m_shiftData = shiftData;
  }
}