// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ScorePositions;
import frc.robot.enums.ScoringHeight;
import frc.robot.subsystems.Arms;

public class ScorePieceHighCone extends CommandBase {
  private Arms m_arms;
  private double rotatePosition;
  private int m_scoringSlot;
  private boolean isCone;
  private ScoringHeight m_scoringHeight;

  /** Creates a new ScorePiece. */
  public ScorePieceHighCone(Arms arms) {
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
boolean executeHigh1 = false;
    if(isCone && m_scoringHeight == ScoringHeight.High){
      m_arms.moveArmRotatePIDPosition(ScorePositions.coneMedHeight, true);
      executeHigh1 = true;
    }
    SmartDashboard.putBoolean("ExecuteHigh1", executeHigh1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    return (isCone && m_scoringHeight == ScoringHeight.High && m_arms.isArmRotateAtPosition()) 
          || isCone == false || m_scoringHeight != ScoringHeight.High;
  }
}