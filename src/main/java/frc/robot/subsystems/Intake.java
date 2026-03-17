package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CANConstants;
import frc.robot.constants.IntakeConstants;

public class Intake extends SubsystemBase {
  private final TalonFX m_intakeWheelsMotor =
      new TalonFX(CANConstants.intakeWheelsMotor, CANConstants.mechanismCanivore);
  private final VelocityVoltage m_velocityVoltage = new VelocityVoltage(0).withSlot(0);
  private final DutyCycleOut m_dutyCycleOut = new DutyCycleOut(0);

  public Intake() {
    TalonFXConfiguration intakeWheelsConfiguration = new TalonFXConfiguration();

    Slot0Configs intakeWheelsSlot0Configs = new Slot0Configs();

    intakeWheelsSlot0Configs.kP = IntakeConstants.kP;
    intakeWheelsSlot0Configs.kI = IntakeConstants.kI;
    intakeWheelsSlot0Configs.kD = IntakeConstants.kD;
    intakeWheelsSlot0Configs.kS = IntakeConstants.kS;
    intakeWheelsSlot0Configs.kV = IntakeConstants.kV;
    intakeWheelsSlot0Configs.kA = IntakeConstants.kA;

    intakeWheelsConfiguration.Slot0 = intakeWheelsSlot0Configs;

    m_intakeWheelsMotor.getConfigurator().apply(intakeWheelsConfiguration);

    StatusCode status = m_intakeWheelsMotor.getConfigurator().apply(intakeWheelsConfiguration);
    if (!status.isOK()) {
      DriverStation.reportError("Failed to apply Intake TalonFX config: " + status, false);
    }
  }

  protected Command runIntake() {
    return runOnce(() -> setIntakeSpeed(IntakeConstants.INTAKE_SPEED));
  }

  protected Command runOuttake() {
    return runOnce(() -> setIntakeSpeed(IntakeConstants.OUTTAKE_SPEED));
  }

  protected Command stopIntake() {
    return runOnce(() -> setPowerZero());
  }

  private void setPowerZero() {
    m_intakeWheelsMotor.setControl(m_dutyCycleOut.withOutput(0));
  }

  private void setIntakeSpeed(AngularVelocity speed) {
    m_intakeWheelsMotor.setControl(m_velocityVoltage.withVelocity(speed));
  }
}

