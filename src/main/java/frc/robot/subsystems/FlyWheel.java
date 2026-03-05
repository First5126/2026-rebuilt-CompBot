package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CANConstants;
import frc.robot.constants.FlyWheelConstants;

public class FlyWheel extends SubsystemBase {

  private TalonFX m_shooterMotor =
      new TalonFX(CANConstants.flyWheelMotor, new CANBus("Canivore 2"));

  private VelocityVoltage m_shooterSpeed = new VelocityVoltage(0);
  private DutyCycleOut m_dutyCycleOut = new DutyCycleOut(0);

  public FlyWheel() {
    TalonFXConfiguration flyWheelConfiguration = new TalonFXConfiguration();

    Slot0Configs slot0 = new Slot0Configs();
    slot0.kP = FlyWheelConstants.kP;

    flyWheelConfiguration.Slot0 = slot0;

    m_shooterMotor.getConfigurator().apply(flyWheelConfiguration);

    SmartDashboard.putNumber("Shooter Speed (MPS)", 40);
  }

  public Command startSpinning() {
    return runOnce(() -> startMotors());
  }

  public Command stopSpinning() {
    return runOnce(() -> stopMotors());
  }

  private void startMotors() {
    m_shooterMotor.setControl(
        m_shooterSpeed.withVelocity(SmartDashboard.getNumber("Shooter Speed (MPS)", 40)));
  }

  private void stopMotors() {
    m_shooterMotor.setControl(m_dutyCycleOut.withOutput(0));
  }
}
