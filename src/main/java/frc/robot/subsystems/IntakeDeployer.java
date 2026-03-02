package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CANConstants;
import frc.robot.constants.IntakeDeployerConstants;

public class IntakeDeployer extends SubsystemBase {

  private TalonFX m_intakeDeployerMotor = new TalonFX(CANConstants.intakeDeloyerMotor);
  final MotionMagicExpoVoltage m_request = new MotionMagicExpoVoltage(0);

  public IntakeDeployer() {
    TalonFXConfiguration m_intakeDeployerConfiguration = new TalonFXConfiguration();

    Slot0Configs m_intakeDeployerSlot0Configs = new Slot0Configs();
    m_intakeDeployerSlot0Configs.kP = IntakeDeployerConstants.intakeKP;

    m_intakeDeployerConfiguration.Slot0 = m_intakeDeployerSlot0Configs;

    MotionMagicConfigs motionMagic = m_intakeDeployerConfiguration.MotionMagic;

    motionMagic.MotionMagicCruiseVelocity = IntakeDeployerConstants.CRUISE_VELOCITY;
    motionMagic.MotionMagicAcceleration = IntakeDeployerConstants.ACCELERATION;

    m_intakeDeployerMotor.getConfigurator().apply(m_intakeDeployerConfiguration);
  }

  public Command raiseIntakeUpCommand() {
    return run(() -> raiseIntakeUp());
  }

  public Command lowerIntakeDownCommand() {
    return run(() -> lowerIntakeDown());
  }

  private void raiseIntakeUp() {
    rotate(IntakeDeployerConstants.INTAKE_UP);
  }

  private void lowerIntakeDown() {
    rotate(IntakeDeployerConstants.INTAKE_DOWN);
  }

  private void rotate(Angle setpoint) {
    m_intakeDeployerMotor.setControl(m_request.withPosition(setpoint));
  }
}
