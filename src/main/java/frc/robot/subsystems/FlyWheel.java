package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.MotorArrangementValue;

import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CANConstants;
import frc.robot.constants.FlyWheelConstants;

public class FlyWheel extends SubsystemBase {

    private TalonFXS m_flyWheelMotor = new TalonFXS(CANConstants.flyWheelMotor);
    private VelocityVoltage m_velocityVoltage = new VelocityVoltage(0);
    private DutyCycleOut m_dutyCycle = new DutyCycleOut(0);

    public FlyWheel() {

        TalonFXSConfiguration talonFXSConfiguration = new TalonFXSConfiguration();
        talonFXSConfiguration.Commutation.MotorArrangement = MotorArrangementValue.Disabled;

        Slot0Configs slot0Configs = new Slot0Configs();
        slot0Configs.kP = FlyWheelConstants.kP;

        m_flyWheelMotor.getConfigurator().apply(talonFXSConfiguration);
    }

    public Command startFlyWheel() {
        return runOnce(() -> {
            m_flyWheelMotor.setControl(m_velocityVoltage.withVelocity(FlyWheelConstants.flyWheelSpeed));
        });
    }

    public Command stopFlyWheel() {
        return runOnce(() -> {
            m_flyWheelMotor.setControl(m_dutyCycle.withOutput(0));
        });
    }



}
