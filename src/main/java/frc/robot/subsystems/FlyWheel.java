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

    SmartDashboard.putNumber("Set Shooter Speed (MPS)", 5);
  }

  public Command setSpeedSolution(Supplier<ShootingSolution> solution) {
    return runOnce(() -> {
        setSpeedControl(solution.get().getFlyWheelSpeed());
    });
  }

  public Command setSpeed(Supplier<LinearVelocity> ballSpeed) {
    return runOnce(() -> setSpeedControl(ballSpeed.get()));
  }

  public Command stopSpinning() {
    return runOnce(() -> stopMotors());
  }

  public LinearVelocity getDashboardSpeed() {
    return MetersPerSecond.of(SmartDashboard.getNumber("Set Shooter Speed (MPS)", 0));
  }

  public LinearVelocity getSpeed() {
    return  MetersPerSecond.of((m_shooterMotor.getVelocity().getValue().in(RadiansPerSecond) / FlyWheelConstants.gearRatio) * FlyWheelConstants.radius.in(Meters));
  }

  private void setSpeedControl(LinearVelocity ballSpeed) {
    AngularVelocity motorSpeed = calculateAngularVelocity(ballSpeed);

    SmartDashboard.putNumber("Calculated Shooter Speed RPS", motorSpeed.in(RotationsPerSecond));
    m_shooterMotor.setControl(m_shooterSpeed.withVelocity(motorSpeed));
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
}
