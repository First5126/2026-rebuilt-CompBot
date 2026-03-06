package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CANConstants;
import frc.robot.constants.IntakeDeployerConstants;

public class IntakeDeployer extends SubsystemBase {

  private TalonFX m_intakeDeployerMotorRight =
      new TalonFX(CANConstants.intakePiviotRight, CANConstants.Canivore2);
  private TalonFX m_intakeDeployerMotorLeft =
      new TalonFX(CANConstants.intakePiviotLeft, CANConstants.Canivore2);
  private TalonFX m_intakeRollerMotor =
      new TalonFX(CANConstants.intakeWheelsMotor, CANConstants.Canivore2);
  final MotionMagicExpoVoltage m_request = new MotionMagicExpoVoltage(0);
  private final VelocityVoltage m_VelocityVoltage =  new VelocityVoltage(0).withSlot(0);

  public IntakeDeployer() {
    TalonFXConfiguration m_intakeDeployerConfiguration = new TalonFXConfiguration();

    Slot0Configs m_intakeDeployerSlot0Configs = new Slot0Configs();
    m_intakeDeployerSlot0Configs.kP = IntakeDeployerConstants.intakeKP;

    m_intakeDeployerConfiguration.Slot0 = m_intakeDeployerSlot0Configs;
    m_intakeDeployerConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    m_intakeDeployerConfiguration.Feedback.SensorToMechanismRatio = 18;

    MotionMagicConfigs motionMagic = m_intakeDeployerConfiguration.MotionMagic;

    motionMagic.MotionMagicCruiseVelocity = IntakeDeployerConstants.CRUISE_VELOCITY;
    motionMagic.MotionMagicAcceleration = IntakeDeployerConstants.ACCELERATION;

    m_intakeDeployerMotorRight.getConfigurator().apply(m_intakeDeployerConfiguration);

    m_intakeDeployerMotorLeft.setControl(new Follower(6, MotorAlignmentValue.Aligned));

    m_intakeRollerMotor.getConfigurator().apply(new TalonFXConfiguration() {{
            MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
            Slot0.kP = 0.3;
          }
        });
  }

  public Command raiseIntakeUpCommand() {
    return runOnce(() -> raiseIntakeUp());
  }

  public Command lowerIntakeDownCommand() {
    return runOnce(() -> lowerIntakeDown());
  }

  public Command runIntakeWheelsCommand() {
    return runOnce(() -> runIntakeWheels());
  }

  public Command runOuttakeWheelsCommand() {
    return runOnce(() -> runOuttakeWheels());
  } 

  public Command stopIntakeWheelsCommand() {
    return runOnce(() -> stopIntakeWheels());
  }

  private void raiseIntakeUp() {
    rotate(IntakeDeployerConstants.INTAKE_UP);
  }

  private void lowerIntakeDown() {
    rotate(IntakeDeployerConstants.INTAKE_DOWN);
  }

  private void runIntakeWheels() {
    m_intakeRollerMotor.setControl(m_VelocityVoltage.withVelocity(IntakeDeployerConstants.INTAKE_SPEED));  
  }

  private void runOuttakeWheels() {
    m_intakeRollerMotor.setControl(m_VelocityVoltage.withVelocity(IntakeDeployerConstants.OUTTAKE_SPEED));
    
  }

  private void stopIntakeWheels() {
    m_intakeRollerMotor.setControl(m_VelocityVoltage.withVelocity(0));
  }

  private void rotate(Angle setpoint) {
    m_intakeDeployerMotorRight.setControl(m_request.withPosition(setpoint));
  }

  private void rollIntakeWheels() {
    
  }
}
