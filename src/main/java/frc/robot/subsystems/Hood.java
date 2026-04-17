// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volt;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.DigitalInputsConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.S1CloseStateValue;
import com.ctre.phoenix6.signals.S2CloseStateValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotLogger;
import frc.robot.constants.CANConstants;
import frc.robot.constants.HoodConstants;
import frc.robot.subsystems.ShootingMechanism.ShootingSolution;
import java.util.function.Supplier;

/**
 * Hood mechanism subsystem.
 *
 * <p>Controls the hood position for aiming and exposes commands for manual and automated
 * positioning.
 */
public class Hood extends SubsystemBase {
  private TalonFX m_hoodMotor;
  private CANdi m_CANdi;
  private CANcoder m_hoodCANCoder;

  private Trigger m_zeroTrigger;

  private Slot0Configs m_motorConfigs;

  private PositionVoltage m_positionVoltageRequest;
  private VoltageOut m_voltageOut = new VoltageOut(0);
  private static final RobotLogger logger = new RobotLogger("Hood");

  private Angle clampPosition(final Angle position) {
    double minDegrees = HoodConstants.MIN_ANGLE.in(Degrees);
    double maxDegrees = HoodConstants.MAX_ANGLE.in(Degrees);
    double requestedDegrees = position.in(Degrees);

    double clampedDegrees = Math.max(minDegrees, Math.min(requestedDegrees, maxDegrees));
    return Degrees.of(clampedDegrees);
  }

  private void setPositionInternal(final Angle position) {
    m_hoodMotor.setControl(m_positionVoltageRequest.withPosition(clampPosition(position)));
  }

  /**
   * Constructs the Hood subsystem and initializes motor, encoder, and limit switch configuration.
   */
  public Hood() {
    m_hoodMotor = new TalonFX(CANConstants.hoodMotor, CANConstants.mechanismCanivore);
    m_CANdi = new CANdi(CANConstants.hoodCANdi, CANConstants.mechanismCanivore);

    m_hoodCANCoder = new CANcoder(CANConstants.hoodEncoder, CANConstants.mechanismCanivore);

    CANcoderConfiguration encoderConfiguration = new CANcoderConfiguration();
    encoderConfiguration.MagnetSensor.MagnetOffset = HoodConstants.MAGNETIC_OFFSET;

    m_hoodCANCoder.getConfigurator().apply(encoderConfiguration);

    // Mechanism Configuration
    TalonFXConfiguration talonConfiguration = new TalonFXConfiguration();
    talonConfiguration.Feedback.RotorToSensorRatio = 8.33;
    talonConfiguration.Feedback.SensorToMechanismRatio = 18;
    talonConfiguration.Feedback.withFusedCANcoder(m_hoodCANCoder);

    m_hoodMotor.getConfigurator().apply(talonConfiguration);

    // PID Configs
    m_motorConfigs = new Slot0Configs();

    m_motorConfigs.kP = HoodConstants.kP;
    m_motorConfigs.kI = HoodConstants.kI;
    m_motorConfigs.kD = HoodConstants.kD;
    m_motorConfigs.kS = HoodConstants.kS;
    m_motorConfigs.kV = HoodConstants.kV;

    m_hoodMotor.getConfigurator().apply(m_motorConfigs);

    // Hardware Limit Switches
    HardwareLimitSwitchConfigs hardConfigs = new HardwareLimitSwitchConfigs();
    hardConfigs.withForwardLimitRemoteCANdiS2(m_CANdi);
    hardConfigs.withForwardLimitEnable(true);

    hardConfigs.withReverseLimitRemoteCANdiS1(m_CANdi);
    hardConfigs.withReverseLimitEnable(true);

    m_hoodMotor.getConfigurator().apply(hardConfigs);

    // Software Limit Switches
    SoftwareLimitSwitchConfigs softConfigs = new SoftwareLimitSwitchConfigs();
    softConfigs.ForwardSoftLimitEnable = true;
    softConfigs.ForwardSoftLimitThreshold = HoodConstants.MAX_ANGLE.in(Rotations);

    softConfigs.ReverseSoftLimitEnable = true;
    softConfigs.ReverseSoftLimitThreshold = HoodConstants.MIN_ANGLE.in(Rotations);

    m_hoodMotor.getConfigurator().apply(softConfigs);

    // CANdi Configs
    DigitalInputsConfigs inputConfigs = new DigitalInputsConfigs();
    inputConfigs.withS1CloseState(S1CloseStateValue.CloseWhenLow);
    inputConfigs.withS2CloseState(S2CloseStateValue.CloseWhenLow);

    m_CANdi.getConfigurator().apply(inputConfigs);

    // Initalize the PositionVoltage request
    m_positionVoltageRequest = new PositionVoltage(0).withSlot(0);

    m_zeroTrigger = new Trigger(this::getLowerLimitValue);
    m_zeroTrigger.onTrue(
        runOnce(
                () -> {
                  m_hoodCANCoder.setPosition(Rotations.of(0.0));
                })
            .ignoringDisable(true));

    m_hoodCANCoder.setPosition(0);
  }

  private boolean getLowerLimitValue() {
    return m_CANdi.getS1Closed().getValue();
  }

  @Override
  public void periodic() {
    double currentAngle = m_hoodMotor.getPosition().getValueAsDouble() * 360.0;
    logger.logAndDisplay("Hood Angle (deg)", currentAngle);
  }

  /**
   * Lowers the hood using a fixed voltage until the lower limit switch is reached, then stops.
   *
   * @return Command sequence that performs the auto-zero routine
   */
  public Command lowerHoodUntilZero() {
    return setVoltage(HoodConstants.VOLTAGE_AUTO_ZERO)
        .andThen(Commands.waitUntil(m_zeroTrigger).andThen(setVoltage(Volt.of(0))));
  }

  public Command manualRotation(Angle amountOfRotation) {
    return runOnce(
        () -> {
          setPositionInternal(m_hoodMotor.getPosition().getValue().plus(amountOfRotation));
        });
  }

  /**
   * Returns a one-shot command to adjust the hood by the provided delta angle.
   *
   * @param amountOfRotation Angle to add to current hood position
   * @return Command that updates the hood position once
   */
  public Command setPosition(Angle angle) {
    return runOnce(
        () -> {
          setPositionInternal(angle);
        });
  }

  /**
   * Returns a command that moves the hood to the given absolute angle.
   *
   * @param angle Target hood {@link Angle}
   * @return Command that sets the hood position once
   */
  public Command moveAngleUpCommand() {
    return Commands.run(
        () -> {
          setPositionInternal(Degrees.of(m_hoodMotor.getPosition().getValue().in(Degrees) + 0.1));
        },
        this);
  }

  /**
   * Returns a command that nudges the hood slightly up while held/executing.
   *
   * @return Command that increments the hood angle
   */
  public Command moveAngleDownCommand() {
    return Commands.run(
        () -> {
          setPositionInternal(Degrees.of(m_hoodMotor.getPosition().getValue().in(Degrees) - 0.1));
        },
        this);
  }

  /**
   * Returns a command that nudges the hood slightly down while held/executing.
   *
   * @return Command that decrements the hood angle
   */
  public Command manualRotationWithSticks(Supplier<Double> controlerY) {
    return run(
        () -> {
          m_hoodCANCoder.setControl(
              m_voltageOut.withOutput(HoodConstants.MAX_VOLTAGE_MANUAL * (controlerY.get() - 0.1)));
        });
  }

  /**
   * Returns a command for manual hood control driven by a joystick supplier.
   *
   * @param controlerY supplier for manual input
   * @return Command that applies manual voltage while running
   */
  public Command holdCertainPosition(Angle angle) {
    return run(() -> {
          setPositionInternal(angle);
        })
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
  }

  /**
   * Holds the hood at a fixed absolute angle until interrupted.
   *
   * @param angle Target angle to hold
   * @return Command that continuously enforces the angle
   */
  public Command setPosition(Supplier<ShootingSolution> shootingSolution) {
    return run(
        () -> {
          setPositionInternal(shootingSolution.get().getPredictedHoodAngle());
        });
  }

  /**
   * Sets the hood position from a {@link ShootingSolution} supplier (used by higher-level
   * commands).
   *
   * @param shootingSolution Supplier that provides a {@link ShootingSolution}
   * @return Command that applies the supplied position
   */
  public Command setPositionToDashboard() {
    return runOnce(
        () -> {
          Angle angle = Degrees.of(SmartDashboard.getNumber("Set Hood Angle (Deg)", 0));
          setPositionInternal(angle);
        });
  }

  /**
   * Sets the hood based on a dashboard-provided angle (one-shot).
   *
   * @return Command that reads dashboard and sets the hood
   */
  public Command setVoltage(Voltage voltage) {
    return runOnce(
        () -> {
          m_hoodMotor.setControl(m_voltageOut.withOutput(voltage));
        });
  }

  /**
   * Applies a direct voltage to the hood motor (one-shot). Useful for auto-zero or manual override.
   *
   * @param voltage Voltage to apply
   * @return Command that applies the voltage
   */
  public Angle getPosition() {
    return m_hoodCANCoder.getPosition().getValue();
  }

  /**
   * Returns the current absolute hood position as reported by the CANcoder.
   *
   * @return Current hood {@link Angle}
   */
}
