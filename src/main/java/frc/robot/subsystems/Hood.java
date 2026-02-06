// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CANConstants;
import frc.robot.constants.HoodConstants;

public class Hood extends SubsystemBase {
  private TalonFX m_hoodMotor;
  private Slot0Configs m_motorConfigs;

  private PositionVoltage m_positionVoltageRequest;

  public Hood() {
    m_hoodMotor = new TalonFX(CANConstants.hoodMotor);

    m_motorConfigs = new Slot0Configs();
    // Set the PID values
    m_motorConfigs.kP = HoodConstants.kP;
    m_motorConfigs.kI = HoodConstants.kI;
    m_motorConfigs.kD = HoodConstants.kD;
    m_motorConfigs.kV = HoodConstants.kV;

    m_hoodMotor.getConfigurator().apply(m_motorConfigs);

    m_positionVoltageRequest = new PositionVoltage(null).withSlot(0);
  }

  public void setPosition(Angle position) {
    m_hoodMotor.setControl(m_positionVoltageRequest.withPosition(position));
  }

  
}
