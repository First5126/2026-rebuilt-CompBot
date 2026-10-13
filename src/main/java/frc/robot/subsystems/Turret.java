package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotLogger;
import frc.robot.constants.CANConstants;
import frc.robot.constants.TurretConstants;
import frc.robot.subsystems.ShootingMechanism.ShootingSolution;
import java.util.function.Supplier;

public class Turret extends SubsystemBase {
  private static final double LOG_PERIOD_SECONDS = 0.1;
  private static final RobotLogger logger = new RobotLogger("Turret");
  private final TalonFXS m_turretMotor;
  // private final CANcoder m_turretEncoder;
  private final PositionVoltage m_positionControl;
  private double m_dynamicOffsetDegrees = 0.0;
  private double m_lastCommandedAngleDeg = 0.0;
  private double m_lastLogSeconds = 0.0;

  public Turret() {
    m_turretMotor = new TalonFXS(CANConstants.turretMotor, CANConstants.mechanismCanivore);
    /*
    m_turretEncoder = new CANcoder(CANConstants.turretEncoder, CANConstants.mechanismCanivore);
    m_turretEncoder.getConfigurator().apply(new CANcoderConfiguration());

    double absolutePosition = m_turretEncoder.getAbsolutePosition().getValue().in(Rotations);
    double magnetOffsetRot = -absolutePosition; // desiredZero (0) - current

    CANcoderConfiguration canCoderConfiguration = new CANcoderConfiguration();
    canCoderConfiguration.MagnetSensor.withSensorDirection(
        SensorDirectionValue.CounterClockwise_Positive);
    canCoderConfiguration.MagnetSensor.withMagnetOffset(magnetOffsetRot);

    m_turretEncoder.getConfigurator().apply(canCoderConfiguration);
    */

    TalonFXSConfiguration talonFXSConfiguration = new TalonFXSConfiguration();
    talonFXSConfiguration.Commutation.MotorArrangement = MotorArrangementValue.NEO550_JST;
    talonFXSConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    talonFXSConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    /*
    talonFXSConfiguration.ExternalFeedback.withSyncCANcoder(m_turretEncoder);
    talonFXSConfiguration.ExternalFeedback.RotorToSensorRatio = 10;
    talonFXSConfiguration.ExternalFeedback.SensorToMechanismRatio = 10;
    */

    CurrentLimitsConfigs limitsConfigs = new CurrentLimitsConfigs();
    // limitsConfigs.SupplyCurrentLimit = TurretConstants.CURRENT_LIMIT;
    limitsConfigs.StatorCurrentLimit = TurretConstants.STATOR_LIMIT;
    // limitsConfigs.SupplyCurrentLimitEnable = true;
    limitsConfigs.StatorCurrentLimitEnable = true;

    talonFXSConfiguration.CurrentLimits = limitsConfigs;

    talonFXSConfiguration.SoftwareLimitSwitch.withForwardSoftLimitThreshold(
        TurretConstants.MAX_ANGLE.times(TurretConstants.GEAR_RATIO));
    talonFXSConfiguration.SoftwareLimitSwitch.withForwardSoftLimitEnable(true);
    talonFXSConfiguration.SoftwareLimitSwitch.withReverseSoftLimitThreshold(
        TurretConstants.MIN_ANGLE.times(TurretConstants.GEAR_RATIO));
    talonFXSConfiguration.SoftwareLimitSwitch.withReverseSoftLimitEnable(true);

    talonFXSConfiguration.ExternalFeedback.FeedbackRemoteSensorID = CANConstants.turretEncoder;

    Slot0Configs slotConfigs = new Slot0Configs();
    slotConfigs.kP = TurretConstants.kP;
    slotConfigs.kI = TurretConstants.kI;
    slotConfigs.kD = TurretConstants.kD;
    slotConfigs.kS = TurretConstants.kS;
    slotConfigs.kV = TurretConstants.kV;
    slotConfigs.kA = TurretConstants.kA;

    talonFXSConfiguration.Slot0 = slotConfigs;

    m_turretMotor.getConfigurator().apply(talonFXSConfiguration);
    // m_turretMotor.setPosition(m_turretEncoder.getPosition().getValue());
    m_positionControl = new PositionVoltage(0);
    m_turretMotor.setPosition(0);
  }

  public Command rotateToPosition(Supplier<ShootingSolution> shootingSolution) {
    return runOnce(
        () -> {
          setPosition(shootingSolution.get().getPredictedTurretAngle());
        });
  }

  public Command rotateToPositionAutoCont(Supplier<ShootingSolution> shootingSolution) {
    return run(() -> {
          setPosition(shootingSolution.get().getPredictedTurretAngle());
        })
        .repeatedly();
  }

  @Override
  public void periodic() {
    double currentAngle = m_turretMotor.getPosition().getValueAsDouble() * 360.0;
    double currentAngleDeg = currentAngle / TurretConstants.GEAR_RATIO;

    logger.log("Angle (deg)", currentAngleDeg);
    logger.log("Commanded Angle (deg)", m_lastCommandedAngleDeg);
    logger.log("Dynamic Offset (deg)", m_dynamicOffsetDegrees);

    double nowSeconds = Timer.getFPGATimestamp();
    if (nowSeconds - m_lastLogSeconds >= LOG_PERIOD_SECONDS) {
      m_lastLogSeconds = nowSeconds;
      logger.logAndDisplay("Angle (deg)", currentAngleDeg);
      logger.logAndDisplay("Commanded Angle (deg)", m_lastCommandedAngleDeg);
    }
  }

  public double findTimeFromFuelShootingDistance(double distance) {
    throw new UnsupportedOperationException(
        "TODO: Implement findTimeFromFuelShootingDistance(double distance) based on shooter model\"");
  }

  public Angle getPosition() {
    return m_turretMotor.getPosition().getValue();
  }

  public Command holdCertainPosition(Angle angle) {
    return run(
        () -> {
          setPosition(angle);
        });
  }

  private void incrementDynamicOffset(final double offsetDegrees) {
    m_dynamicOffsetDegrees += offsetDegrees;
  }

  public Command adjustPositionDynamically(double offsetDegrees) {
    return runOnce(
        () -> {
          incrementDynamicOffset(offsetDegrees);
        });
  }

  private void setPosition(Angle position) {
    position = position.times(TurretConstants.GEAR_RATIO);
    // Convert all angles to degrees for clamping
    double minDegrees = TurretConstants.MIN_ANGLE.in(Degrees) * TurretConstants.GEAR_RATIO;
    double maxDegrees = TurretConstants.MAX_ANGLE.in(Degrees) * TurretConstants.GEAR_RATIO;
    double requestedDegrees =
        position.in(Degrees) + m_dynamicOffsetDegrees * TurretConstants.GEAR_RATIO;
    double clampedDegrees = Math.max(minDegrees, Math.min(requestedDegrees, maxDegrees));

    // Never command more than the full physical travel from the current position, so a wrapped
    // bearing can not demand a rotation through the mechanism zero at full speed.
    double currentDegrees = m_turretMotor.getPosition().getValue().in(Degrees);
    double maxStepDegrees = TurretConstants.MAX_ANGLE.in(Degrees) * TurretConstants.GEAR_RATIO;
    clampedDegrees =
        Math.max(
            currentDegrees - maxStepDegrees,
            Math.min(clampedDegrees, currentDegrees + maxStepDegrees));

    m_lastCommandedAngleDeg = clampedDegrees / TurretConstants.GEAR_RATIO;

    // Construct the measure back in degrees
    Angle clampedPosition = Degrees.of(clampedDegrees);

    m_turretMotor.setControl(m_positionControl.withPosition(clampedPosition));
  }
  /*
  public void resetEncoder() {
    // Recompute the magnet offset so the CANcoder reads logical zero (0 rotations)
    CANcoderConfiguration canCoderConfiguration = new CANcoderConfiguration();
    double absRot = m_turretEncoder.getAbsolutePosition().getValue().in(Rotations);
    double magnetOffsetRot = -absRot;
    canCoderConfiguration.MagnetSensor.withMagnetOffset(magnetOffsetRot);
    m_turretEncoder.getConfigurator().apply(canCoderConfiguration);
  }
    */
}
