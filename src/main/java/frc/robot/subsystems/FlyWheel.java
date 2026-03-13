package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.constants.AprilTagLocalizationConstants.MAX_TAG_DISTANCE;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.CANConstants;
import frc.robot.constants.FlyWheelConstants;
import frc.robot.subsystems.ShootingMechanism.ShootingSolution;

import java.util.function.DoubleSupplier;
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

    SmartDashboard.putNumber("Set Shooter Speed (RPS)", 5);
  }

  public Command setSpeed(Supplier<AngularVelocity> rps) {
    return runOnce(() -> {
        AngularVelocity speed = rps.get();
        setSpeedControl(speed);
    });
  }

  public Command setSpeedWithSolution(Supplier<ShootingSolution> solutionSupplier) {
    return runOnce(() -> {
        ShootingSolution solution = solutionSupplier.get();
        setSpeedControl(solution.predictedFlyWheelVelocity);
    });
  }

  public Command rotateFlywheel() {
    return runOnce(() -> startMotors());
  }

  public Command stopSpinning() {
    return runOnce(() -> stopMotors());
  }

  // public LinearVelocity getDashboardSpeed() {
  //  return MetersPerSecond.of(SmartDashboard.getNumber("Set Shooter Speed (MPS)", 0));
  // }

  public AngularVelocity getDashboardSpeedRPS() {
    return RotationsPerSecond.of(SmartDashboard.getNumber("Set Shooter Speed (RPS)", 0));
  }

  public AngularVelocity getCurrentSpeed() {
    return m_shooterMotor.getVelocity().getValue();
  }

  /*private void setSpeedControl(Supplier<LinearVelocity> ballSpeed) {
    AngularVelocity motorSpeed = calculateAngularVelocity(ballSpeed.get());

    SmartDashboard.putNumber("Calculated Shooter Speed RPS", motorSpeed.in(RotationsPerSecond));
    m_shooterMotor.setControl(m_shooterSpeed.withVelocity(motorSpeed));
  }*/

  private void setSpeedControl(AngularVelocity rotationSpeed) {

    if (rotationSpeed.isEquivalent(RotationsPerSecond.of(0))) stopMotors();
    else m_shooterMotor.setControl(m_shooterSpeed.withVelocity(rotationSpeed));
  }

  private void startMotors() {
    m_shooterMotor.setControl(m_dutyCycleOut.withOutput(0.60));
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
