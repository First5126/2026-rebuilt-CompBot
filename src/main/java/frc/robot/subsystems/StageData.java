// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;

public class StageData extends SubsystemBase {
  private static char m_firstInactiveAlliance;
  private static char m_alliance;

  /** Initializes cached game-specific stage data. */
  public StageData() {
    m_firstInactiveAlliance = ' ';
    m_alliance = ' ';
  }

  /** Updates cached alliance and stage data from DriverStation. */
  @Override
  public void periodic() {
    if (m_alliance == ' ') {
      Optional<DriverStation.Alliance> our_alliance = DriverStation.getAlliance();
      if (our_alliance.isPresent()) {
        m_alliance = our_alliance.get().name().charAt(0);
      }
    }

    if (m_firstInactiveAlliance == ' ') {
      String gameData = DriverStation.getGameSpecificMessage();
      if (gameData.length() > 0) {
        m_firstInactiveAlliance = gameData.charAt(0);
      }
    }
  }

  public enum GameStage {
    Auto,
    TransitionShift,
    ShiftOne,
    ShiftTwo,
    ShiftThree,
    ShiftFour,
    Endgame
  }

  /**
   * Returns the current game stage based on match time.
   *
   * @return current game stage
   */
  public static GameStage getStage() {
    boolean auto = DriverStation.isAutonomous();
    if (auto) return GameStage.Auto;

    double gameTime = DriverStation.getMatchTime();
    if (gameTime < 0) return GameStage.TransitionShift;
    if (gameTime < 30) return GameStage.Endgame;
    else if (gameTime < 55) return GameStage.ShiftFour;
    else if (gameTime < 80) return GameStage.ShiftThree;
    else if (gameTime < 105) return GameStage.ShiftTwo;
    else if (gameTime < 130) return GameStage.ShiftOne;
    else return GameStage.TransitionShift;
  }

  /**
   * Determines if scoring is allowed for the current stage and alliance.
   *
   * @return true if scoring is allowed
   */
  public static boolean canScore() {
    GameStage stage = getStage();

    if (stage == GameStage.Auto || stage == GameStage.Endgame) return true;
    else if (stage == GameStage.ShiftOne || stage == GameStage.ShiftThree) {
      if (m_alliance == m_firstInactiveAlliance) return false;
      else return true;
    } else if (stage == GameStage.ShiftTwo || stage == GameStage.ShiftFour) {
      if (m_alliance == m_firstInactiveAlliance) return true;
      else return false;
    } else return false;
  }
}
