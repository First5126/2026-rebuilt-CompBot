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
import frc.robot.constants.IntakeConstants;

public class Intake extends SubsystemBase {

  private TalonFX m_intakeDeployerMotorRight =
      new TalonFX(CANConstants.intakeWheelsMotor, CANConstants.mechanismCanivore);
  private TalonFX m_intakeDeployerMotorLeft =
      new TalonFX(CANConstants.intakeWheelsMotor, CANConstants.mechanismCanivore);
  private TalonFX m_intakeRollerMotor =
      new TalonFX(CANConstants.intakeWheelsMotor, CANConstants.mechanismCanivore);
  final MotionMagicExpoVoltage m_request = new MotionMagicExpoVoltage(0);
  private final VelocityVoltage m_VelocityVoltage =  new VelocityVoltage(0).withSlot(0);

  public Intake() {
    TalonFXConfiguration m_intakeDeployerConfiguration = new TalonFXConfiguration();

    Slot0Configs m_intakeDeployerSlot0Configs = new Slot0Configs();
    m_intakeDeployerSlot0Configs.kP = IntakeConstants.intakeKP;

    m_intakeDeployerConfiguration.Slot0 = m_intakeDeployerSlot0Configs;
    m_intakeDeployerConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    m_intakeDeployerConfiguration.Feedback.SensorToMechanismRatio = 18;

    MotionMagicConfigs motionMagic = m_intakeDeployerConfiguration.MotionMagic;

    motionMagic.MotionMagicCruiseVelocity = IntakeConstants.CRUISE_VELOCITY;
    motionMagic.MotionMagicAcceleration = IntakeConstants.ACCELERATION;

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
    Command outtakeWheels = runOuttakeWheelsCommand();
    outtakeWheels.addRequirements(this);
    return outtakeWheels;
  } 

  public Command stopIntakeWheelsCommand() {
    return runOnce(() -> stopIntakeWheels());
  }

  private void raiseIntakeUp() {
    rotate(IntakeConstants.INTAKE_UP);
  }

  private void lowerIntakeDown() {
    rotate(IntakeConstants.INTAKE_DOWN);
  }

  private void runIntakeWheels() {
    m_intakeRollerMotor.setControl(m_VelocityVoltage.withVelocity(IntakeConstants.INTAKE_SPEED));  
  }

  private void runOuttakeWheels() {
    m_intakeRollerMotor.setControl(m_VelocityVoltage.withVelocity(IntakeConstants.OUTTAKE_SPEED));
    
  }

  private void stopIntakeWheels() {
    m_intakeRollerMotor.setControl(m_VelocityVoltage.withVelocity(0));
  }

  private void rotate(Angle setpoint) {
    m_intakeDeployerMotorRight.setControl(m_request.withPosition(setpoint));
  }
}