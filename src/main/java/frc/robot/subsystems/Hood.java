// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CANConstants;
import frc.robot.constants.HoodConstants;

public class Hood extends SubsystemBase {
  private TalonFX m_hoodMotor;
  private TalonFXConfiguration m_motorConfigs;

  private PositionVoltage m_positionVoltageRequest;

  private final CANcoder m_hoodEncoder =
      new CANcoder(CANConstants.hoodEncoder, CANConstants.driveBaseCanivore);

  public Hood() {
    m_hoodMotor = new TalonFX(CANConstants.hoodMotor);

    CANcoderConfiguration canCoderConfiguration = new CANcoderConfiguration();
    canCoderConfiguration.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    m_hoodEncoder.getConfigurator().apply(canCoderConfiguration);

    m_motorConfigs = new TalonFXConfiguration();

    // m_motorConfigs.HardwareLimitSwitch.ForwardLimitRemoteSensorID =

    // Set the PID values
    m_motorConfigs.Slot0.kP = HoodConstants.kP;
    m_motorConfigs.Slot0.kI = HoodConstants.kI;
    m_motorConfigs.Slot0.kD = HoodConstants.kD;
    m_motorConfigs.Slot0.kV = HoodConstants.kV;

    m_hoodMotor.getConfigurator().apply(m_motorConfigs);

    m_positionVoltageRequest = new PositionVoltage(null).withSlot(0);
  }

  public Command setPosition(Angle angle) {
    return runOnce(
        () -> {
          m_hoodMotor.setControl(m_positionVoltageRequest.withPosition(angle));
        });
  }
}
