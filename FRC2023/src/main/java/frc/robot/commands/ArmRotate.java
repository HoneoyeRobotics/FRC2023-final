// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotPrefs;
import frc.robot.subsystems.Arms;

public class ArmRotate extends CommandBase {
  private Arms m_arms;
  private boolean up;
  private double m_speed;
  /** Creates a new RotateArm. */
  public ArmRotate(Arms arms, boolean up) {
    m_arms = arms;
    this.up = up;
   // addRequirements(arms);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(up)
      m_speed = RobotPrefs.getArmRotateUpSpeed();
    else
      m_speed = RobotPrefs.getArmRotateDownSpeed();
    // m_arms.armRotateBrakeOff();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_arms.isArmRotateIPDEnabled()){
        
      double movement = RobotPrefs.getArmRotatePIDMovement() * (up ? 1 : -1);
      m_arms.moveArmRotatePIDPosition(movement, false);
    }
    else{
      m_arms.rotateArm(m_speed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arms.rotateArm(0.0);
    // m_arms.armRotateBrakeOn();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}