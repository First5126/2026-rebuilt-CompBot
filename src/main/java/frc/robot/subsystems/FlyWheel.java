package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotLogger;
import frc.robot.constants.CANConstants;
import frc.robot.constants.FlyWheelConstants;
import frc.robot.subsystems.ShootingMechanism.ShootingSolution;
import java.util.function.Supplier;

public class FlyWheel extends SubsystemBase {

  private static final RobotLogger logger = new RobotLogger("FlyWheel");

  private TalonFX m_shooterMotor =
      new TalonFX(CANConstants.flyWheelsMotor, CANConstants.mechanismCanivore);

  private VelocityVoltage m_shooterSpeed = new VelocityVoltage(0);
  private DutyCycleOut m_dutyCycleOut = new DutyCycleOut(0);
  private AngularVelocity m_lastTargetSpeed = RotationsPerSecond.of(0);

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

  public Command setSpeedWithSolution(Supplier<ShootingSolution> solutionSupplier) {
    return run(
        () -> {
          ShootingSolution solution = solutionSupplier.get();
          setSpeedControl(solution.getPredictedFlyWheelVelocity());
        });
  }

  public Command stopSpinning() {
    return run(() -> stopMotors());
  }

  public Command stopSpinningOnce() {
    return runOnce(() -> stopMotors());
  }

  public AngularVelocity getCurrentSpeed() {
    return m_shooterMotor.getVelocity().getValue();
  }

  @Override
  public void periodic() {
    logger.log("Current Speed (rps)", m_shooterMotor.getVelocity().getValue().in(RotationsPerSecond));
    logger.log("Target Speed (rps)", m_lastTargetSpeed.in(RotationsPerSecond));
  }

  private void setSpeedControl(AngularVelocity rotationSpeed) {

    m_lastTargetSpeed = rotationSpeed;
    if (rotationSpeed.isEquivalent(RotationsPerSecond.of(0))) stopMotors();
    else m_shooterMotor.setControl(m_shooterSpeed.withVelocity(rotationSpeed));
  }

  private void stopMotors() {
    m_lastTargetSpeed = RotationsPerSecond.of(0);
    m_shooterMotor.setControl(m_dutyCycleOut.withOutput(0));
  }

  public Command shootInCommand() {
    return runOnce(
        () -> {
          m_shooterMotor.setControl(m_dutyCycleOut.withOutput(-0.30));
        });
  }
}