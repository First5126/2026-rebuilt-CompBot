// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  /* log and replay timestamp and joystick data */
  private final HootAutoReplay m_timeAndJoystickReplay =
      new HootAutoReplay().withTimestampReplay().withJoystickReplay();

  /** Creates the robot and initializes the container. */
  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  /** Runs once per robot packet. */
  @Override
  public void robotPeriodic() {
    m_timeAndJoystickReplay.update();
    CommandScheduler.getInstance().run();
  }

  /** Called once when the robot transitions to disabled. */
  @Override
  public void disabledInit() {}

  /** Called periodically while disabled. */
  @Override
  public void disabledPeriodic() {}

  /** Called once when leaving disabled mode. */
  @Override
  public void disabledExit() {}

  /** Called once when autonomous mode starts. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  /** Called periodically during autonomous mode. */
  @Override
  public void autonomousPeriodic() {}

  /** Called once when autonomous mode ends. */
  @Override
  public void autonomousExit() {}

  /** Called once when teleop mode starts. */
  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().cancel(m_autonomousCommand);
    }
  }

  /** Called periodically during teleop mode. */
  @Override
  public void teleopPeriodic() {}

  /** Called once when teleop mode ends. */
  @Override
  public void teleopExit() {}

  /** Called once when test mode starts. */
  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  /** Called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** Called once when test mode ends. */
  @Override
  public void testExit() {}

  /** Called periodically during simulation. */
  @Override
  public void simulationPeriodic() {}
}
