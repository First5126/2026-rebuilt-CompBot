package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotLogger;
import frc.robot.constants.CANConstants;
import frc.robot.constants.TurretConstants;
import frc.robot.subsystems.ShootingMechanism.ShootingSolution;
import java.util.function.Supplier;

/**
 * Turret mechanism subsystem.
 *
 * <p>Controls turret motor and exposes commands to move to specific angles, manual control, and
 * dynamic offset adjustments.
 */
public class Turret extends SubsystemBase {
  private static final RobotLogger logger = new RobotLogger("Turret");
  private final TalonFXS m_turretMotor;
  // private final CANcoder m_turretEncoder;
  private final PositionVoltage m_positionControl;
  private final VoltageOut m_voltageControl;
  private double m_dynamicOffsetDegrees = 0.0;

  /** Constructs the Turret subsystem and configures motor, limits, and control parameters. */
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
    m_voltageControl = new VoltageOut(0);
    m_turretMotor.setPosition(0);
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

  /**
   * Returns a command to manually drive the turret using a supplier value (typically joystick).
   *
   * @param controlerX Supplier providing the manual control input ([-1..1])
   * @return Command that applies voltage proportional to the supplied input
   */
  public Command manualRotationWithSticks(Supplier<Double> controlerX) {
    return run(
        () -> {
          m_turretMotor.setControl(
              m_voltageControl.withOutput(
                  TurretConstants.MAX_VOLTAGE_MANUAL * (controlerX.get() - 0.1)));
        });
  }

  /**
   * Returns a one-shot command that rotates the turret to the position provided by the shooting
   * solution supplier.
   *
   * @param shootingSolution supplier of {@link ShootingSolution}
   * @return Command that sets turret position once
   */
  public Command rotateToPosition(Supplier<ShootingSolution> shootingSolution) {
    return runOnce(
        () -> {
          setPosition(shootingSolution.get().getPredictedTurretAngle());
        });
  }

  /**
   * Returns a command which continuously rotates the turret to follow a supplied shooting solution.
   *
   * @param shootingSolution Supplier of {@link ShootingSolution}
   * @return Command that applies the turret setpoint each execution
   */
  public Command rotateToPositionTracking(Supplier<ShootingSolution> shootingSolution) {
    return run(
        () -> {
          setPosition(shootingSolution.get().getPredictedTurretAngle());
        });
  }

  /**
   * Continuously rotates the turret to follow the supplied shooting solution (auto mode).
   *
   * @param shootingSolution Supplier of {@link ShootingSolution}
   * @return Command that repeatedly applies the shooting solution's turret setpoint
   */
  public Command rotateToPositionAuto(Supplier<ShootingSolution> shootingSolution) {
    return run(
        () -> {
          setPosition(shootingSolution.get().getPredictedTurretAngle());
        });
  }

  /**
   * Returns a one-shot command that rotates the turret to zero (0 degrees).
   *
   * @return Command that sets turret to zero position
   */
  public Command rotateToZero() {
    return runOnce(
        () -> {
          setPosition(Degrees.of(0));
        });
  }

  /**
   * Returns a command which continuously applies the supplied shooting solution's turret setpoint
   * (repeating).
   *
   * @param shootingSolution supplier of {@link ShootingSolution}
   * @return repeating Command that tracks the provided solution
   */
  public Command rotateToPositionAutoCont(Supplier<ShootingSolution> shootingSolution) {
    return run(() -> {
          setPosition(shootingSolution.get().getPredictedTurretAngle());
        })
        .repeatedly();
  }

  /**
   * Returns a command that sets the logical current position as zero on the hardware.
   *
   * @return Command that zeroes the turret encoder/position
   */
  public Command setZero() {
    return runOnce(
        () -> {
          m_turretMotor.setPosition(0);
        });
  }

  @Override
  public void periodic() {
    double currentAngle = m_turretMotor.getPosition().getValueAsDouble() * 360.0;
    logger.logAndDisplay("Turret Angle (deg)", currentAngle / TurretConstants.GEAR_RATIO);
  }

  /**
   * Computes a time-of-flight estimate for a given shooting distance.
   *
   * @param distance Distance to target in meters
   * @return Time in seconds that the projectile will be in flight
   * @throws UnsupportedOperationException if not implemented
   */
  public double findTimeFromFuelShootingDistance(double distance) {
    throw new UnsupportedOperationException(
        "TODO: Implement findTimeFromFuelShootingDistance(double distance) based on shooter model\"");
  }

  /**
   * Returns the current turret position as reported by the motor controller.
   *
   * @return Current turret {@link edu.wpi.first.units.measure.Angle}
   */
  public Angle getPosition() {
    return m_turretMotor.getPosition().getValue();
  }

  /**
   * Checks whether the turret is within a tolerance of a target position.
   *
   * @param targetPosition Target angle
   * @param tolerance Allowed angular tolerance
   * @return true if current position is near target within tolerance
   */
  public boolean isAtPosition(Angle targetPosition, Angle tolerance) {
    return getPosition().isNear(targetPosition, tolerance);
  }

  public Command holdCertainPosition(Angle angle) {
    return run(
        () -> {
          setPosition(angle);
        });
  }

  /**
   * Adjusts the turret offset by the specified degrees in a one-shot command.
   *
   * @param offsetDegrees Degrees to add to the dynamic offset
   * @return Command that updates the dynamic offset once
   */
  public Command adjustPositionDynamically(double offsetDegrees) {
    return runOnce(
        () -> {
          incrementDynamicOffset(offsetDegrees);
        });
  }

  private void incrementDynamicOffset(final double offsetDegrees) {
    m_dynamicOffsetDegrees += offsetDegrees;
  }

  private void resetDynamicOffset() {
    m_dynamicOffsetDegrees = 0.0;
  }

  /**
   * Returns a command that resets the turret's dynamic offset value to zero.
   *
   * @return Command that resets the dynamic offset
   */
  public Command resetDynamicOffsetCommand() {
    return run(
        () -> {
          resetDynamicOffset();
        });
  }

  private void setPosition(Angle position) {
    position = position.times(TurretConstants.GEAR_RATIO);
    // Convert all angles to degrees for clamping
    double minDegrees = TurretConstants.MIN_ANGLE.in(Degrees) * TurretConstants.GEAR_RATIO;
    double maxDegrees = TurretConstants.MAX_ANGLE.in(Degrees) * TurretConstants.GEAR_RATIO;
    double requestedDegrees = position.in(Degrees) + m_dynamicOffsetDegrees;
    double clampedDegrees = Math.max(minDegrees, Math.min(requestedDegrees, maxDegrees));
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
