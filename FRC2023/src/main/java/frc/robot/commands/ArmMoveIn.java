// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotPrefs;
import frc.robot.subsystems.Arms;

public class ArmMoveIn extends CommandBase {
  /** Creates a new MoveArmIn. */
  private Arms arms;
  private int i;

  public ArmMoveIn(Arms arms) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arms = arms;
    // addRequirements(arms);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    i = 0;
    arms.armLengthBrakeOff();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arms.moveArmInOut(-1 * RobotPrefs.getArmLengthInSpeed());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arms.armLengthBrakeOn();
    arms.moveArmInOut(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (arms.isArmIn()) {
      arms.armLengthBrakeOn();
      i++;
    }
    return (i >= 5) || arms.armLengthOverload();
  }
}