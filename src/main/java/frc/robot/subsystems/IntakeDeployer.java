package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CANConstants;
import frc.robot.constants.IntakeDeployerConstants;

public class IntakeDeployer extends SubsystemBase {

  private TalonFX m_intakeDeployerMotorRight =
      new TalonFX(CANConstants.intakePivotRightMotor, CANConstants.mechanismCanivore);
  private TalonFX m_intakeDeployerMotorLeft =
      new TalonFX(CANConstants.intakePivotLeftMotor, CANConstants.mechanismCanivore);
  final MotionMagicExpoVoltage m_request = new MotionMagicExpoVoltage(0);
  final DutyCycleOut m_dutyRequest = new DutyCycleOut(0);

  public IntakeDeployer() {
    TalonFXConfiguration m_intakeDeployerConfiguration = new TalonFXConfiguration();

    SmartDashboard.getBoolean("ReachedLowerSetpoint", reachedDeploySetpoint());

    Slot0Configs m_intakeDeployerSlot0Configs = new Slot0Configs();
    m_intakeDeployerSlot0Configs.kP = IntakeDeployerConstants.INTAKE_KP;
    m_intakeDeployerSlot0Configs.kG = IntakeDeployerConstants.INTAKE_KG;

    m_intakeDeployerConfiguration.Slot0 = m_intakeDeployerSlot0Configs;
    m_intakeDeployerConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    m_intakeDeployerConfiguration.Feedback.SensorToMechanismRatio =
        IntakeDeployerConstants.GEAR_RATIO;
    m_intakeDeployerConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Hardware Limit Switches
    HardwareLimitSwitchConfigs hardConfigs = new HardwareLimitSwitchConfigs();
    hardConfigs.ForwardLimitSource = ForwardLimitSourceValue.LimitSwitchPin;
    hardConfigs.withForwardLimitEnable(true);

    hardConfigs.ReverseLimitSource = ReverseLimitSourceValue.LimitSwitchPin;
    hardConfigs.withReverseLimitEnable(true);

    hardConfigs.ForwardLimitAutosetPositionEnable = true;

    hardConfigs.ReverseLimitAutosetPositionValue = -0.184814;
    hardConfigs.ReverseLimitAutosetPositionEnable = true;

    m_intakeDeployerConfiguration.HardwareLimitSwitch = hardConfigs;

    MotionMagicConfigs motionMagic = m_intakeDeployerConfiguration.MotionMagic;

    motionMagic.MotionMagicCruiseVelocity = IntakeDeployerConstants.CRUISE_VELOCITY;
    motionMagic.MotionMagicAcceleration = IntakeDeployerConstants.ACCELERATION;

    m_intakeDeployerMotorLeft.getConfigurator().apply(m_intakeDeployerConfiguration);

    m_intakeDeployerMotorRight.setControl(
        new Follower(m_intakeDeployerMotorLeft.getDeviceID(), MotorAlignmentValue.Aligned));
  }

  public Command raiseIntakeUpCommand() {
    return runOnce(() -> raiseIntakeUp());
  }

  public Command agitateIntakeUp() {
    return runOnce(() -> rotateToAgitation());
  }

  // public Command agitateIntakeCommand() {
  // return run(() -> agitateIntake()).repeatedly();
  // }

  public Command agitateIntakeCo() {
    Command agitateUp = agitateIntakeUp();

    return agitateUp
        .until(this::reachedAgitateSetpoint)
        .andThen(Commands.waitSeconds(0.25))
        .andThen(
            lowerIntakeDownCommand()
                .until(this::reachedDeploySetpoint)
                .andThen(Commands.waitSeconds(0.25)))
        .repeatedly();
  }

  public Command lowerIntakeDownCommand() {
    return setWheelsDownCommand()
        .andThen(Commands.waitUntil(this::reachedDeploySetpoint))
        .andThen(stopWheelsCommand());
  }

  private Boolean reachedDeploySetpoint() {
    return m_intakeDeployerMotorLeft.getPosition().getValue().in(Rotations)
        > IntakeDeployerConstants.INTAKE_HALFWAY_DOWN.in(Rotations);
  }

  private Boolean reachedAgitateSetpoint() {
    return m_intakeDeployerMotorLeft.getPosition().getValue().in(Rotations)
        < IntakeDeployerConstants.MAX_AGITATION_HEIGHT.in(Rotations);
  }

  private Command setWheelsDownCommand() {
    return runOnce(() -> lowerIntakeHalfwayDown());
  }

  private Command stopWheelsCommand() {
    return runOnce(() -> stopMotor());
  }

  private void raiseIntakeUp() {
    rotate(IntakeDeployerConstants.INTAKE_UP);
  }

  private void lowerIntakeHalfwayDown() {
    rotate(IntakeDeployerConstants.INTAKE_HALFWAY_DOWN);
  }

  private void rotateToAgitation() {
    rotate(IntakeDeployerConstants.MAX_AGITATION_HEIGHT);
  }

  private void lowerIntakeDown() {
    rotate(IntakeDeployerConstants.INTAKE_DOWN);
  }

  public Command lowerIntakeDownFullyCommand() {
    return runOnce(() -> lowerIntakeDown());
  }

  private void rotate(Angle setpoint) {
    m_intakeDeployerMotorLeft.setControl(m_request.withPosition(setpoint));
  }

  private void stopMotor() {
    m_intakeDeployerMotorLeft.setControl(m_dutyRequest.withOutput(0));
  }
}
