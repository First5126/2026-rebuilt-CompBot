package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CANConstants;
import frc.robot.constants.IntakeDeployerConstants;

public class IntakeDeployer extends SubsystemBase {

  private TalonFX m_intakeDeployerMotorRight =
      new TalonFX(CANConstants.intakePivotRightMotor, CANConstants.mechanismCanivore);
  private TalonFX m_intakeDeployerMotorLeft =
      new TalonFX(CANConstants.intakePivotLeftMotor, CANConstants.mechanismCanivore);
  final MotionMagicExpoVoltage m_request = new MotionMagicExpoVoltage(0);

  public IntakeDeployer() {
    TalonFXConfiguration m_intakeDeployerConfiguration = new TalonFXConfiguration();

    Slot0Configs m_intakeDeployerSlot0Configs = new Slot0Configs();
    m_intakeDeployerSlot0Configs.kP = IntakeDeployerConstants.intakeKP;

    m_intakeDeployerConfiguration.Slot0 = m_intakeDeployerSlot0Configs;
    m_intakeDeployerConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    m_intakeDeployerConfiguration.Feedback.SensorToMechanismRatio = 18;
    m_intakeDeployerConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    MotionMagicConfigs motionMagic = m_intakeDeployerConfiguration.MotionMagic;

    motionMagic.MotionMagicCruiseVelocity = IntakeDeployerConstants.CRUISE_VELOCITY;
    motionMagic.MotionMagicAcceleration = IntakeDeployerConstants.ACCELERATION;

    m_intakeDeployerMotorRight.getConfigurator().apply(m_intakeDeployerConfiguration);
    m_intakeDeployerMotorLeft.getConfigurator().apply(m_intakeDeployerConfiguration);

    m_intakeDeployerMotorLeft.setControl(
        new Follower(m_intakeDeployerMotorRight.getDeviceID(), MotorAlignmentValue.Aligned));
  }

  public Command raiseIntakeUpCommand() {
    return runOnce(() -> raiseIntakeUp());
  }

  public Command lowerIntakeDownCommand() {
    return runOnce(() -> lowerIntakeDown());
  }

  private void raiseIntakeUp() {
    rotate(IntakeDeployerConstants.INTAKE_UP);
  }

  private void lowerIntakeDown() {
    rotate(IntakeDeployerConstants.INTAKE_DOWN);
  }

  private void rotate(Angle setpoint) {
    m_intakeDeployerMotorRight.setControl(m_request.withPosition(setpoint));
  }
}
