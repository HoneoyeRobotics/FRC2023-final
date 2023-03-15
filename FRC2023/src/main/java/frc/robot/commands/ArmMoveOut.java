// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotPrefs;
import frc.robot.subsystems.Arms;

public class ArmMoveOut extends CommandBase {
  /** Creates a new MoveArmOut. */
  private Arms arms;

  public ArmMoveOut(Arms arms) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arms = arms;
    addRequirements(arms);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arms.armLengthBrakeOff();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arms.moveArmInOut(1 * RobotPrefs.getArmLengthOutSpeed());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arms.moveArmInOut(0);
    arms.armLengthBrakeOn();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return arms.isArmOut() || arms.armLengthOverload();
  }
}