// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotPrefs;
import frc.robot.subsystems.Arms;

public class ArmMoveCompletelyIn extends CommandBase {
  private Arms m_arms;

  /** Creates a new MoveArmCompletelyIn. */
  public ArmMoveCompletelyIn(Arms arms) {
    m_arms = arms;
    addRequirements(m_arms);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_arms.armLengthBrakeOff();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_arms.moveArmInCompletely(-1 * RobotPrefs.getArmLengthInSpeed());
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
    if (m_arms.isArmIn())
      m_arms.resetArmLengthEncoder();
    return m_arms.isArmIn() || m_arms.armLengthOverload();
  }
}