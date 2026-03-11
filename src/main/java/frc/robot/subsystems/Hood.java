// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ReverseLimitValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.CANConstants;
import frc.robot.constants.HoodConstants;

public class Hood extends SubsystemBase {
  private TalonFX m_hoodMotor;
  private CANdi m_CANdi;
  private CANcoder m_hoodCANCoder;

  private Trigger m_zeroTrigger;

  private Slot0Configs m_motorConfigs;

  private PositionVoltage m_positionVoltageRequest;

  public Hood() {
    m_hoodMotor = new TalonFX(CANConstants.hoodMotor, CANConstants.mechanismCanivore);
    m_CANdi = new CANdi(CANConstants.hoodCANdi, CANConstants.mechanismCanivore);
    
    m_hoodCANCoder = new CANcoder(CANConstants.hoodEncoder, CANConstants.mechanismCanivore);
    
    TalonFXConfiguration talonConfiguration = new TalonFXConfiguration();
    talonConfiguration.Feedback.RotorToSensorRatio = 8.33;
    talonConfiguration.Feedback.SensorToMechanismRatio = 18;
    talonConfiguration.Feedback.withFusedCANcoder(
      m_hoodCANCoder);
      
    m_hoodMotor.getConfigurator().apply(talonConfiguration);
    
    // PID Configs
    m_motorConfigs = new Slot0Configs();
    
    m_motorConfigs.kP = HoodConstants.kP;
    m_motorConfigs.kI = HoodConstants.kI;
    m_motorConfigs.kD = HoodConstants.kD;
    m_motorConfigs.kS = HoodConstants.kS;
    m_motorConfigs.kV = HoodConstants.kV;
    
    m_hoodMotor.getConfigurator().apply(m_motorConfigs);

    // Hardware Limit Switches
    /*HardwareLimitSwitchConfigs limitSwitchConfigs = new HardwareLimitSwitchConfigs();
    limitSwitchConfigs.withReverseLimitRemoteCANdiS2(m_CANdi);
    limitSwitchConfigs.withForwardLimitRemoteCANdiS1(m_CANdi);
    
    limitSwitchConfigs.withReverseLimitAutosetPositionEnable(true);
    limitSwitchConfigs.withReverseLimitAutosetPositionValue(Rotations.of(0));
    
    m_hoodMotor.getConfigurator().apply(limitSwitchConfigs);*/
    
    // Initalize the PositionVoltage request
    m_positionVoltageRequest = new PositionVoltage(0).withSlot(0);
    
    m_zeroTrigger = new Trigger(this::getLowerLimitValue);
    m_zeroTrigger.onTrue(runOnce(
      () -> {m_hoodCANCoder.setPosition(Rotations.of(0.0));})
      .ignoringDisable(true));
  }

  private boolean getLowerLimitValue() {
    return m_CANdi.getS1Closed().getValue();
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean(
        "Reverse Limit",
        m_hoodMotor.getReverseLimit().getValue() == ReverseLimitValue.ClosedToGround);
    SmartDashboard.putBoolean(
        "Forward Limit",
        m_hoodMotor.getReverseLimit().getValue() == ReverseLimitValue.ClosedToGround);
  }

  public Command setPosition(Angle angle) {
    return runOnce(
        () -> {
          m_hoodMotor.setControl(m_positionVoltageRequest.withPosition(angle));
        });
  }
}
