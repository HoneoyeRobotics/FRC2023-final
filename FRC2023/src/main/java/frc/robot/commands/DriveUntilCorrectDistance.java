// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.enums.LimeLightState;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Vision;

public class DriveUntilCorrectDistance extends CommandBase {
  private DriveTrain m_DriveTrain;
  private Vision m_Vision;

  /** Creates a new DriveUntilCorrectDistance. */
  public DriveUntilCorrectDistance(DriveTrain driveTrain, Vision vision) {
    m_DriveTrain = driveTrain;
    m_Vision = vision;

    addRequirements(m_DriveTrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Vision.setFrontLimelightState(LimeLightState.AprilTag);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_DriveTrain.arcadeDrive(.5, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_DriveTrain.arcadeDrive(0, 0);
    m_Vision.setFrontLimelightState(LimeLightState.Drive);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_Vision.correctDistance();
  }
}
