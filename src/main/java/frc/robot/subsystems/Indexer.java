package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CANConstants;
import frc.robot.constants.IndexerConstants;

/**
 * Indexer subsystem responsible for feeding game pieces into the shooter.
 *
 * <p>Provides commands to start, stop, reverse, and agitate the indexing mechanisms.
 */
public class Indexer extends SubsystemBase {

  private TalonFX m_indexerMotor =
      new TalonFX(CANConstants.indexFeederMotor, CANConstants.mechanismCanivore);
  private TalonFX m_spindexerMotor =
      new TalonFX(CANConstants.spindexerMotor, CANConstants.mechanismCanivore);

  private VoltageOut m_indexerVoltageOut = new VoltageOut(0);
  private VoltageOut m_spindexerVoltageOut = new VoltageOut(0);
  private DutyCycleOut m_dutyCycleOut = new DutyCycleOut(0);

  public Indexer() {

    TalonFXConfiguration m_indexerConfiguration = new TalonFXConfiguration();
    m_indexerConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    Slot0Configs m_indexerSlot0Configs = new Slot0Configs();
    m_indexerSlot0Configs.kP = IndexerConstants.indexerKP;

    m_indexerConfiguration.Slot0 = m_indexerSlot0Configs;

    m_indexerMotor.getConfigurator().apply(m_indexerConfiguration);

    TalonFXConfiguration m_spindexerConfiguration = new TalonFXConfiguration();
    m_spindexerConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    Slot0Configs m_spindexerSlot0Configs = new Slot0Configs();
    m_spindexerSlot0Configs.kP = IndexerConstants.indexerKP;

    m_spindexerConfiguration.Slot0 = m_spindexerSlot0Configs;

    m_spindexerMotor.getConfigurator().apply(m_spindexerConfiguration);
  }

  public Command startIndexing() {
    return runOnce(() -> startMotors());
  }

  /**
   * Starts the indexing motors to feed game pieces into the shooter (one-shot).
   *
   * @return Command that begins indexing
   */
  public Command stopIndexing() {
    return runOnce(() -> stopMotors());
  }

  /**
   * Stops the indexing motors (one-shot).
   *
   * @return Command that stops indexing
   */
  public Command reverseIndexing() {
    return runOnce(() -> reverseMotors());
  }

  /**
   * Reverses the indexer briefly to clear jams (one-shot).
   *
   * @return Command that runs indexer in reverse
   */
  public Command agitateIndexer() {
    return startIndexing()
        .andThen(
            Commands.waitSeconds(1).andThen(reverseIndexing().andThen(Commands.waitSeconds(0.1))));
  }

  /**
   * Runs an agitate routine which briefly indexes forward then reverses to help clear jams.
   *
   * @return Composite command that agitates the indexer
   */
  private void startMotors() {
    m_indexerMotor.setControl(m_indexerVoltageOut.withOutput(IndexerConstants.indexerSpeed));
    m_spindexerMotor.setControl(m_spindexerVoltageOut.withOutput(IndexerConstants.spindexerSpeed));
  }

  private void stopMotors() {
    m_indexerMotor.setControl(m_dutyCycleOut.withOutput(0));
    m_spindexerMotor.setControl(m_dutyCycleOut.withOutput(0));
  }

  private void reverseMotors() {
    m_indexerMotor.setControl(m_indexerVoltageOut.withOutput(IndexerConstants.indexerSpeedReverse));
    m_spindexerMotor.setControl(
        m_spindexerVoltageOut.withOutput(IndexerConstants.spindexerSpeedReverse));
  }
}
