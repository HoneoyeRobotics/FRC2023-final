// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotPrefs;
import frc.robot.Constants.ArmLength;
import frc.robot.Constants.ScorePositions;
import frc.robot.enums.ScoringHeight;
import frc.robot.subsystems.Arms;

public class ScorePiece2 extends CommandBase {
  private Arms m_arms;
  private double lengthPosition;
  private int m_scoringSlot;
  private boolean isCone;
  private ScoringHeight m_scoringHeight;
  /** Creates a new ScorePiece. */
  public ScorePiece2(Arms arms) {
    m_arms = arms;

    addRequirements(m_arms);
    // Use addRequirements() here to declare subsystem dependencies.
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_scoringSlot = m_arms.getScoringSlot();
    m_scoringHeight = m_arms.getScoringHeight();

    isCone = m_arms.isCone(m_scoringSlot);

    if(m_scoringHeight == ScoringHeight.Low) {
      lengthPosition = ScorePositions.lowLength;
    }
    else {
      if(m_scoringHeight == ScoringHeight.Med) {
        if(isCone) {
          lengthPosition = ScorePositions.coneMedLength;
        }
        else {
          lengthPosition = ScorePositions.cubeMedLength;
        }
        }
      else {
        if(isCone) {
          lengthPosition = ScorePositions.coneHighLength;
        }
        else {
          lengthPosition = ScorePositions.cubeHighLength;
        }
      }
    }
    m_arms.armLengthBrakeOff();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      m_arms.moveArmToPosition(lengthPosition);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arms.armLengthBrakeOn();
    m_arms.moveArmInOut(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (((m_arms.armLengthMotorCurrentPosition() + ArmLength.deadband) > lengthPosition 
          && (m_arms.armLengthMotorCurrentPosition() - ArmLength.deadband) < lengthPosition)
          || m_arms.isArmOut());
  }
}