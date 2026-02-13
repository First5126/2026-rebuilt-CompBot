package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CANConstants;
import frc.robot.constants.IntakeConstants;

public class Intake extends SubsystemBase {
  
  private TalonFXS m_intakeMotor = new TalonFXS(CANConstants.intakeMotor);;
  private VelocityVoltage m_velocityVoltage = new VelocityVoltage(0);
  private DutyCycleOut m_dutyCycleOut = new DutyCycleOut(0);

  public Intake() {
        TalonFXSConfiguration talonFXSConfiguration = new TalonFXSConfiguration();
        talonFXSConfiguration.Commutation.MotorArrangement = MotorArrangementValue.Disabled; // TODO: change this
        talonFXSConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        Slot0Configs slot0Configs = new Slot0Configs();
        slot0Configs.kP = IntakeConstants.kP;

        talonFXSConfiguration.Slot0 = slot0Configs;

        m_intakeMotor.getConfigurator().apply(talonFXSConfiguration);
  }

  public Command startIntake() {
      return run(
          () -> {
            setSpeed(IntakeConstants.intakeSpeed);
      });
  }

  public Command stopIntake() {
      return run(
          () -> {
            stop();
      });
  }
  
  private void setSpeed(AngularVelocity speed) {
    m_intakeMotor.setControl(m_velocityVoltage.withVelocity(speed).withSlot(0));
  }

  private void stop() {
    m_intakeMotor.setControl(m_dutyCycleOut.withOutput(0));
  }

}
