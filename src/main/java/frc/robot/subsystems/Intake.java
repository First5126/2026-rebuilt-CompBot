package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CANConstants;
import frc.robot.constants.IntakeConstants;

public class Intake extends SubsystemBase {
  private TalonFX m_intakeWheelsMotor =
      new TalonFX(CANConstants.intakeWheelsMotor, CANConstants.mechanismCanivore);
  private final VelocityVoltage m_velocityVoltage = new VelocityVoltage(0).withSlot(0);
  private final DutyCycleOut m_dutyCycleOut = new DutyCycleOut(0);

  public Intake() {
    TalonFXConfiguration m_intakeWheelsConfiguration = new TalonFXConfiguration();

    Slot0Configs m_intakeWheelsSlot0Configs = new Slot0Configs();

    m_intakeWheelsSlot0Configs.kP = IntakeConstants.kP;
    m_intakeWheelsSlot0Configs.kI = IntakeConstants.kI;
    m_intakeWheelsSlot0Configs.kD = IntakeConstants.kD;
    m_intakeWheelsSlot0Configs.kS = IntakeConstants.kS;
    m_intakeWheelsSlot0Configs.kV = IntakeConstants.kV;
    m_intakeWheelsSlot0Configs.kA = IntakeConstants.kA;

    m_intakeWheelsConfiguration.Slot0 = m_intakeWheelsSlot0Configs;

    m_intakeWheelsMotor.getConfigurator().apply(m_intakeWheelsConfiguration);
  }

  public Command runIntakeCommand() {
    return runOnce(() -> runIntake());
  }

  public Command runOuttakeCommand() {
    return runOnce(() -> runOuttake());
  }

  public Command stopIntakeCommand() {
    return runOnce(() -> stopIntake());
  }

  public void runIntake() {
    rollIntake(IntakeConstants.INTAKE_SPEED);
  }

  public void stopIntake() {
    m_intakeWheelsMotor.setControl(m_dutyCycleOut.withOutput(0));
  }

  public void runOuttake() {
    rollIntake(IntakeConstants.OUTTAKE_SPEED);
  }

  private void rollIntake(AngularVelocity speed) {
    m_intakeWheelsMotor.setControl(m_velocityVoltage.withVelocity(speed));
  }
}
