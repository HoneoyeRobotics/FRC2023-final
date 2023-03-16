// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmLength;
import frc.robot.Constants.GrabPositions;
import frc.robot.enums.GrabPosition;
import frc.robot.subsystems.Arms;

public class GrabPiece extends CommandBase {
  private Arms arms;
  private GrabPosition grabPosition;
  private double rotatePosition;
  private double lengthPosition;

  /** Creates a new GrabPiece. */
  public GrabPiece(Arms m_arms) {
    arms = m_arms;
    grabPosition = arms.getGrabPosition();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    switch (grabPosition) {
      case Cube:
        rotatePosition = GrabPositions.cubeHeight;
        lengthPosition = GrabPositions.cubeLength;
        break;
      case ConePointIn:
        rotatePosition = GrabPositions.coneInHeight;
        lengthPosition = GrabPositions.coneInLength;
        break;
      case ConePointUp:
        rotatePosition = GrabPositions.coneUpHeight;
        lengthPosition = GrabPositions.coneUpLength;
        break;
      case ConePointOut:
        rotatePosition = GrabPositions.coneOutHeight;
        lengthPosition = GrabPositions.coneOutLength;
        break;
      default:
        break;
    }
    arms.moveArmRotatePIDPosition(rotatePosition, true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (arms.isArmRotateAtPosition()) {
      arms.armLengthBrakeOff();
      arms.moveArmToPosition(lengthPosition);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arms.moveArmInOut(0.0);
    arms.armLengthBrakeOn();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ((arms.armLengthMotorCurrentPosition() + ArmLength.deadband) > lengthPosition &&
        (arms.armLengthMotorCurrentPosition() - ArmLength.deadband) < lengthPosition);
  }
}