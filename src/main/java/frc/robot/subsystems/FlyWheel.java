package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CANConstants;
import frc.robot.constants.FlyWheelConstants;
import frc.robot.subsystems.ShootingMechanism.ShootingSolution;
import java.util.function.Supplier;

/**
 * Flywheel subsystem that controls shooter motors.
 *
 * <p>Provides commands to spin the flywheel to target speeds and utility methods for current
 * velocity reading.
 */
public class FlyWheel extends SubsystemBase {

  private TalonFX m_shooterMotor =
      new TalonFX(CANConstants.flyWheelsMotor, CANConstants.mechanismCanivore);

  private VelocityVoltage m_shooterSpeed = new VelocityVoltage(0);
  private DutyCycleOut m_dutyCycleOut = new DutyCycleOut(0);

  public FlyWheel() {
    TalonFXConfiguration flyWheelConfiguration = new TalonFXConfiguration();

    flyWheelConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    Slot0Configs slot0 = new Slot0Configs();
    slot0.kP = FlyWheelConstants.kP;
    slot0.kI = FlyWheelConstants.kI;
    slot0.kD = FlyWheelConstants.kD;
    slot0.kS = FlyWheelConstants.kS;
    slot0.kV = FlyWheelConstants.kV;
    slot0.kA = FlyWheelConstants.kA;

    flyWheelConfiguration.Slot0 = slot0;

    m_shooterMotor.getConfigurator().apply(flyWheelConfiguration);
  }

  /** Constructs the FlyWheel subsystem and configures the shooter motor. */
  public Command setSpeed(Supplier<AngularVelocity> rps) {
    return runOnce(
        () -> {
          AngularVelocity speed = rps.get();
          setSpeedControl(speed);
        });
  }

  /**
   * Returns a one-shot command that sets the flywheel speed from the supplied angular velocity.
   *
   * @param rps supplier of angular velocity (RPS)
   * @return Command that sets the flywheel speed once
   */

  /**
   * Returns a command that sets the flywheel speed once from the provided supplier.
   *
   * @param rps Supplier providing target angular velocity for the flywheel
   * @return Command to set the speed once
   */
  public Command setSpeedWithSolution(Supplier<ShootingSolution> solutionSupplier) {
    return run(
        () -> {
          ShootingSolution solution = solutionSupplier.get();
          setSpeedControl(solution.getPredictedFlyWheelVelocity());
        });
  }

  /**
   * Returns a command that continuously sets the flywheel speed according to a provided {@link
   * ShootingSolution} supplier.
   *
   * @param solutionSupplier Supplier of {@link ShootingSolution}
   * @return Command that applies the requested speed on each execution
   */
  public Command rotateFlywheel() {
    return runOnce(() -> startMotors());
  }

  /**
   * Spins up the flywheel (one-shot) using the configured duty/voltage.
   *
   * @return Command that begins wheel rotation
   */
  public Command stopSpinning() {
    return runOnce(() -> stopMotors());
  }

  /**
   * Stops the flywheel motors (one-shot).
   *
   * @return Command that stops motor output
   */
  public Command reverseSpinning() {
    return runOnce(() -> reverseWheels());
  }

  /**
   * Reverses flywheel direction for the duration of a one-shot command.
   *
   * @return Command that runs the flywheel in reverse
   */

  // public LinearVelocity getDashboardSpeed() {
  //  return MetersPerSecond.of(SmartDashboard.getNumber("Set Shooter Speed (MPS)", 0));
  // }

  /**
   * Reads a dashboard-provided target speed (RPS) and returns it as an AngularVelocity.
   *
   * @return Dashboard-configured angular velocity for the shooter
   */
  public AngularVelocity getDashboardSpeedRPS() {
    return RotationsPerSecond.of(SmartDashboard.getNumber("Set Shooter Speed (RPS)", 0));
  }

  /**
   * Returns the current measured angular velocity of the flywheel motor.
   *
   * @return Current flywheel {@link AngularVelocity}
   */
  public AngularVelocity getCurrentSpeed() {
    return m_shooterMotor.getVelocity().getValue();
  }

  /** Periodic no-op for FlyWheel; update logic may be added later. */
  @Override
  public void periodic() {}

  /*private void setSpeedControl(Supplier<LinearVelocity> ballSpeed) {
    AngularVelocity motorSpeed = calculateAngularVelocity(ballSpeed.get());

    SmartDashboard.putNumber("Calculated Shooter Speed RPS", motorSpeed.in(RotationsPerSecond));
    m_shooterMotor.setControl(m_shooterSpeed.withVelocity(motorSpeed));
  }*/

  private void setSpeedControl(AngularVelocity rotationSpeed) {

    if (rotationSpeed.isEquivalent(RotationsPerSecond.of(0))) stopMotors();
    else m_shooterMotor.setControl(m_shooterSpeed.withVelocity(rotationSpeed));
  }

  private void reverseWheels() {
    m_shooterMotor.setControl(m_shooterSpeed.withVelocity(RotationsPerSecond.of(-1)));
  }

  private void startMotors() {
    m_shooterMotor.setControl(m_dutyCycleOut.withOutput(0.60));
  }

  private void setSpeed(AngularVelocity speed) {
    m_shooterMotor.setControl(m_shooterSpeed.withVelocity(speed));
  }

  private void stopMotors() {
    m_shooterMotor.setControl(m_dutyCycleOut.withOutput(0));
  }

  private AngularVelocity calculateAngularVelocity(LinearVelocity linearVelocity) {
    return RadiansPerSecond.of(
        linearVelocity.in(MetersPerSecond)
            / FlyWheelConstants.radius.in(Meters)
            * FlyWheelConstants.gearRatio);
  }

  public Command shootInCommand() {
    return runOnce(
        () -> {
          m_shooterMotor.setControl(m_dutyCycleOut.withOutput(-0.30));
        });
  }
  /**
   * Runs the shooter inward briefly to clear jams (one-shot).
   *
   * @return Command that runs the motor inward once
   */
}
