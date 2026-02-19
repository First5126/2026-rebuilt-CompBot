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
public class Indexer extends SubsystemBase {

    private TalonFXS m_indexerMotor = new TalonFXS(CANConstants.indexerMotor);
    private DutyCycleOut m_dutyCycleOut = new DutyCycleOut(0);
    private VelocityVoltage m_velocityVoltage = new VelocityVoltage(0);

    public Indexer() {

        TalonFXSConfiguration talonFXSConfiguration = new TalonFXSConfiguration();
        talonFXSConfiguration.Commutation.MotorArrangement = MotorArrangementValue.Disabled;

        Slot0Configs slot0Configs = new Slot0Configs();
        slot0Configs.kP = IndexerConstants.kP;

        m_indexerMotor.getConfigurator().apply(talonFXSConfiguration);
    }

    public Command runIndexer() {
        return runOnce(() -> {
            m_indexerMotor.setControl(m_velocityVoltage.withVelocity(IndexerConstants.indexerSpeed));
        });
    }

    public Command stopIndexer() {
        return runOnce(() -> {
            m_indexerMotor.setControl(m_dutyCycleOut.withOutput(0));
        });
    }
}
