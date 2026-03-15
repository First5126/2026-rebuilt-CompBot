package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RevolutionsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.CANConstants;

public class Intake extends SubsystemBase {
    private TalonFX m_intakeWheelsMotor = new TalonFX(CANConstants.intakeWheelsMotor, CANConstants.mechanismCanivore);
    private final VelocityVoltage m_velocityVoltage = new VelocityVoltage(0).withSlot(0);
    private final DutyCycleOut m_dutyCycleOut = new DutyCycleOut(0);

private final VoltageOut m_voltReq = new VoltageOut(0.0);

private final SysIdRoutine m_sysIdRoutine =
   new SysIdRoutine(
      new SysIdRoutine.Config(
         null,        // Use default ramp rate (1 V/s)
         Volts.of(4), // Reduce dynamic step voltage to 4 to prevent brownout
         null,        // Use default timeout (10 s)
                      // Log state with Phoenix SignalLogger class
         (state) -> SignalLogger.writeString("state", state.toString())
      ),
      new SysIdRoutine.Mechanism(
         (volts) -> m_intakeWheelsMotor.setControl(m_voltReq.withOutput(volts.in(Volts))),
         null,
         this
      )
   );

   public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
   return m_sysIdRoutine.quasistatic(direction);
}

public Command sysIdDynamic(SysIdRoutine.Direction direction) {
   return m_sysIdRoutine.dynamic(direction);
}

public Intake() {
    TalonFXConfiguration m_intakeWheelsConfiguration = new TalonFXConfiguration();

    Slot0Configs m_intakeWheelsSlot0Configs = new Slot0Configs();

    m_intakeWheelsSlot0Configs.kP = IntakeConstants.kP;
    m_intakeWheelsSlot0Configs.kI = IntakeConstants.kI;
    m_intakeWheelsSlot0Configs.kD = IntakeConstants.kD;
    m_intakeWheelsSlot0Configs.kS = IntakeConstants.kS;
    m_intakeWheelsSlot0Configs.kV = IntakeConstants.kV;
    m_intakeWheelsSlot0Configs.kA = IntakeConstants.kA;

    m_intakeWheelsConfiguration.Slot0 = m_intakeWheelsSlot0Configs;

    m_intakeWheelsMotor.getConfigurator().apply(m_intakeWheelsConfiguration);

    }

    public Command runIntakeCommand() {
        return runOnce(() -> runIntake());
    }

    public Command runOuttakeCommand() {
        return runOnce(() -> runOuttake());
    }

    public Command stopIntakeCommand() {
        return runOnce(() -> stopIntake());
    }


    public void runIntake() {
        rollIntake(IntakeConstants.INTAKE_SPEED);
    }

    public void stopIntake() {
       m_intakeWheelsMotor.setControl(m_dutyCycleOut.withOutput(0));
    }

    public void runOuttake() {
        rollIntake(IntakeConstants.OUTTAKE_SPEED);
    }

    private void rollIntake(AngularVelocity speed) {
        m_intakeWheelsMotor.setControl(m_velocityVoltage.withVelocity(speed));
  }
}