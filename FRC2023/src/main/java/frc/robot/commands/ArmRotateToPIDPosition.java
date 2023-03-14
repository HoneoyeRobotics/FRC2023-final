// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arms;

public class ArmRotateToPIDPosition extends CommandBase {
  private Arms m_arms;
  private double m_position;
  /** Creates a new RotateArmToPositionPID. */
  public ArmRotateToPIDPosition(Arms arms, double position) {
    m_arms = arms;
    m_position = position;
    //addRequirements(m_arms);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_arms.moveArmRotatePIDPosition(m_position, true);
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
    return m_arms.isArmRotateAtPosition();
    
  }
}