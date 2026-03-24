package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotLogger;
import frc.robot.constants.CANConstants;
import frc.robot.constants.TurretConstants;
import frc.robot.subsystems.ShootingMechanism.ShootingSolution;
import java.util.function.Supplier;

public class Turret extends SubsystemBase {
  private static final RobotLogger logger = new RobotLogger("Turret");
  private final TalonFXS m_turretMotor;
  private final CANcoder m_turretEncoder;
  private final PositionVoltage m_positionControl;
  private final VoltageOut m_voltageControl;

  public Turret() {
    m_turretMotor = new TalonFXS(CANConstants.turretMotor, CANConstants.mechanismCanivore);
    m_turretEncoder = new CANcoder(CANConstants.turretEncoder, CANConstants.mechanismCanivore);

    CANcoderConfiguration canCoderConfiguration = new CANcoderConfiguration();
    canCoderConfiguration.MagnetSensor.withSensorDirection(
        SensorDirectionValue.CounterClockwise_Positive);
    canCoderConfiguration.MagnetSensor.withMagnetOffset(TurretConstants.ENCODER_OFFSET);

    m_turretEncoder.getConfigurator().apply(canCoderConfiguration);

    TalonFXSConfiguration talonFXSConfiguration = new TalonFXSConfiguration();
    talonFXSConfiguration.Commutation.MotorArrangement = MotorArrangementValue.NEO550_JST;
    talonFXSConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    talonFXSConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    talonFXSConfiguration.ExternalFeedback.withFusedCANcoder(m_turretEncoder);
    talonFXSConfiguration.ExternalFeedback.RotorToSensorRatio = 10;
    talonFXSConfiguration.ExternalFeedback.SensorToMechanismRatio = 10;

    CurrentLimitsConfigs limitsConfigs = new CurrentLimitsConfigs();
    // limitsConfigs.SupplyCurrentLimit = TurretConstants.CURRENT_LIMIT;
    limitsConfigs.StatorCurrentLimit = TurretConstants.STATOR_LIMIT;
    // limitsConfigs.SupplyCurrentLimitEnable = true;
    limitsConfigs.StatorCurrentLimitEnable = true;

    talonFXSConfiguration.CurrentLimits = limitsConfigs;

    talonFXSConfiguration.SoftwareLimitSwitch.withForwardSoftLimitThreshold(
        TurretConstants.MAX_ANGLE);
    talonFXSConfiguration.SoftwareLimitSwitch.withForwardSoftLimitEnable(true);
    talonFXSConfiguration.SoftwareLimitSwitch.withReverseSoftLimitThreshold(
        TurretConstants.MIN_ANGLE);
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
    m_positionControl = new PositionVoltage(0);
    m_voltageControl = new VoltageOut(0);
  }

  /**
   * Returns a command that rotates the turret to the specified position.
   *
   * @param position The target angle (use WPILib Units, e.g. Units.Degrees.of(90))
   * @return a WPILib Command object to run once
   */
  public Command manualRotation(Angle amountOfMovement) {
    return run(
        () -> {
          setPosition(m_turretMotor.getPosition().getValue().plus(amountOfMovement));
        });
  }

  public Command manualRotationWithSticks(Supplier<Double> controlerX) {
    return run(
        () -> {
          m_turretMotor.setControl(
              m_voltageControl.withOutput(
                  TurretConstants.MAX_VOLTAGE_MANUAL * (controlerX.get() - 0.1)));
        });
  }

  public Command rotateToPosition(Supplier<ShootingSolution> shootingSolution) {
    return runOnce(
        () -> {
          setPosition(shootingSolution.get().getPredictedTurretAngle());
        });
  }

  public Command rotateToPositionTracking(Supplier<ShootingSolution> shootingSolution) {
    return run(
        () -> {
          setPosition(shootingSolution.get().getPredictedTurretAngle());
        });
  }

  public Command rotateToPositionAuto(Supplier<ShootingSolution> shootingSolution) {
    return run(
        () -> {
          setPosition(shootingSolution.get().getPredictedTurretAngle());
        });
  }

  public Command rotateToZero() {
    return runOnce(
        () -> {
          setPosition(Degrees.of(0));
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
    logger.log("Turret Angle (deg)", currentAngle);
  }

  public double findTimeFromFuelShootingDistance(double distance) {
    throw new UnsupportedOperationException(
        "TODO: Implement findTimeFromFuelShootingDistance(double distance) based on shooter model\"");
  }

  public Angle getPosition() {
    return m_turretMotor.getPosition().getValue();
  }

  public boolean isAtPosition(Angle targetPosition, Angle tolerance) {
    return getPosition().isNear(targetPosition, tolerance);
  }

  public Command holdCertainPosition(Angle angle) {
    return run(
        () -> {
          setPosition(angle);
        });
  }

  private void setPosition(final Angle position) {
    // Convert all angles to degrees for clamping
    double minDegrees = TurretConstants.MIN_ANGLE.in(Degrees);
    double maxDegrees = TurretConstants.MAX_ANGLE.in(Degrees);
    double requestedDegrees = position.in(Degrees);
    double clampedDegrees = Math.max(minDegrees, Math.min(requestedDegrees, maxDegrees));
    // Construct the measure back in degrees
    Angle clampedPosition = Degrees.of(clampedDegrees);

    m_turretMotor.setControl(m_positionControl.withPosition(clampedPosition));
  }
}
