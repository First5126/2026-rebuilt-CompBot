package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.MotorArrangementValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CANConstants;
import frc.robot.constants.IndexerConstants;
public class Spindexer extends SubsystemBase {

    private TalonFXS m_spindexerMotor = new TalonFXS(CANConstants.indexerMotor);
    private DutyCycleOut m_dutyCycleOut = new DutyCycleOut(0);
    private VelocityVoltage m_velocityVoltage = new VelocityVoltage(0);

    public Spindexer() {

        TalonFXSConfiguration talonFXSConfiguration = new TalonFXSConfiguration();
        talonFXSConfiguration.Commutation.MotorArrangement = MotorArrangementValue.Disabled;

        Slot0Configs slot0Configs = new Slot0Configs();
        slot0Configs.kP = IndexerConstants.kP;

        m_spindexerMotor.getConfigurator().apply(talonFXSConfiguration);
    }

    public Command runSpindexer() {
        return runOnce(() -> {
            m_spindexerMotor.setControl(m_velocityVoltage.withVelocity(IndexerConstants.indexerSpeed));
        });
    }

    public Command stopSpindexer() {
        return runOnce(() -> {
            m_spindexerMotor.setControl(m_dutyCycleOut.withOutput(0));
        });
    }
}
