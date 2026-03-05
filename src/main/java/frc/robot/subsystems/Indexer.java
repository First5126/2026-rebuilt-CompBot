package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CANConstants;
import frc.robot.constants.IndexerConstants;

public class Indexer extends SubsystemBase {

  private TalonFX m_indexerMotor = new TalonFX(CANConstants.indexerMotor, new CANBus("Canivore 2"));
  private TalonFX m_spindexerMotor =
      new TalonFX(CANConstants.spindexerMotor, new CANBus("Canivore 2"));

  private VelocityVoltage m_indexerVelocityVoltage = new VelocityVoltage(0);
  private VelocityVoltage m_spindexerVelocityVoltage = new VelocityVoltage(0);
  private DutyCycleOut m_dutyCycleOut = new DutyCycleOut(0);

  public Indexer() {

    TalonFXConfiguration m_indexerConfiguration = new TalonFXConfiguration();

    Slot0Configs m_indexerSlot0Configs = new Slot0Configs();
    m_indexerSlot0Configs.kP = IndexerConstants.indexerKP;

    m_indexerConfiguration.Slot0 = m_indexerSlot0Configs;

    m_indexerMotor.getConfigurator().apply(m_indexerConfiguration);

    TalonFXConfiguration m_spindexerConfiguration = new TalonFXConfiguration();

    Slot0Configs m_spindexerSlot0Configs = new Slot0Configs();
    m_spindexerSlot0Configs.kP = IndexerConstants.indexerKP;

    m_spindexerConfiguration.Slot0 = m_spindexerSlot0Configs;

    m_spindexerMotor.getConfigurator().apply(m_spindexerConfiguration);
  }

  public Command startIndexing() {
    return runOnce(() -> startMotors());
  }

  public Command stopIndexing() {
    return runOnce(() -> stopMotors());
  }

  private void startMotors() {
    m_indexerMotor.setControl(m_indexerVelocityVoltage.withVelocity(IndexerConstants.indexerSpeed));
    m_spindexerMotor.setControl(
        m_spindexerVelocityVoltage.withVelocity(IndexerConstants.spindexerSpeed));
  }

  private void stopMotors() {
    m_indexerMotor.setControl(m_dutyCycleOut.withOutput(0));
    m_spindexerMotor.setControl(m_dutyCycleOut.withOutput(0));
  }
}
