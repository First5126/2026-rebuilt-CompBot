// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.DigitalInputsConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ReverseLimitValue;
import com.ctre.phoenix6.signals.S1CloseStateValue;
import com.ctre.phoenix6.signals.S2CloseStateValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.CANConstants;
import frc.robot.constants.HoodConstants;
import frc.robot.subsystems.ShootingMechanism.ShootingSolution;
import java.util.function.Supplier;

public class Hood extends SubsystemBase {
  private TalonFX m_hoodMotor;
  private CANdi m_CANdi;
  private CANcoder m_hoodCANCoder;

  private Trigger m_zeroTrigger;

  private Slot0Configs m_motorConfigs;

  private PositionVoltage m_positionVoltageRequest;
  private VoltageOut m_voltageOut = new VoltageOut(0);

  public Hood() {
    m_hoodMotor = new TalonFX(CANConstants.hoodMotor, CANConstants.mechanismCanivore);
    m_CANdi = new CANdi(CANConstants.hoodCANdi, CANConstants.mechanismCanivore);

    m_hoodCANCoder = new CANcoder(CANConstants.hoodEncoder, CANConstants.mechanismCanivore);

    // Mechanism Configuration
    TalonFXConfiguration talonConfiguration = new TalonFXConfiguration();
    talonConfiguration.Feedback.RotorToSensorRatio = 8.33;
    talonConfiguration.Feedback.SensorToMechanismRatio = 18;
    talonConfiguration.Feedback.withFusedCANcoder(m_hoodCANCoder);

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
    HardwareLimitSwitchConfigs hardConfigs = new HardwareLimitSwitchConfigs();
    hardConfigs.withForwardLimitRemoteCANdiS2(m_CANdi);
    hardConfigs.withForwardLimitEnable(true);

    hardConfigs.withReverseLimitRemoteCANdiS1(m_CANdi);
    hardConfigs.withReverseLimitEnable(true);

    m_hoodMotor.getConfigurator().apply(hardConfigs);

    // Software Limit Switches
    SoftwareLimitSwitchConfigs softConfigs = new SoftwareLimitSwitchConfigs();
    softConfigs.ForwardSoftLimitEnable = true;
    softConfigs.ForwardSoftLimitThreshold = 0.070;

    softConfigs.ReverseSoftLimitEnable = true;
    softConfigs.ReverseSoftLimitThreshold = 0;

    m_hoodMotor.getConfigurator().apply(softConfigs);

    // CANdi Configs
    DigitalInputsConfigs inputConfigs = new DigitalInputsConfigs();
    inputConfigs.withS1CloseState(S1CloseStateValue.CloseWhenLow);
    inputConfigs.withS2CloseState(S2CloseStateValue.CloseWhenLow);

    m_CANdi.getConfigurator().apply(inputConfigs);

    // Initalize the PositionVoltage request
    m_positionVoltageRequest = new PositionVoltage(0).withSlot(0);

    SmartDashboard.putNumber("Set Hood Angle (Deg)", 0);

    m_zeroTrigger = new Trigger(this::getLowerLimitValue);
    m_zeroTrigger.onTrue(
        runOnce(
                () -> {
                  m_hoodCANCoder.setPosition(Rotations.of(0.0));
                })
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

  public Command setPosition(Supplier<ShootingSolution> shootingSolution) {
    return runOnce(
        () -> {
          m_hoodMotor.setControl(
              m_positionVoltageRequest.withPosition(shootingSolution.get().predictedHoodAngle));
        });
  }

  public Command setPositionToDashboard() {
    return runOnce(
        () -> {
          Angle angle = Degrees.of(SmartDashboard.getNumber("Set Hood Angle (Deg)", 0));
          m_hoodMotor.setControl(
              m_positionVoltageRequest.withPosition(angle));
        });
  }

  public Command setVoltage(Voltage voltage) {
    return runOnce(() -> {
        m_hoodMotor.setControl(m_voltageOut.withOutput(voltage));
    });
  }
}
