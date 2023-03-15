// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arms;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Vision;

public class DriveUntilPerpendicular extends CommandBase {
  private DriveTrain m_driveTrain;
  private Vision m_vision;
  private Arms m_arms;
  private int scoringPositon;
  private int allianceColor = 0;
  private int slot;

  /** Creates a new DriveUntilPerpendicular. */
  public DriveUntilPerpendicular(DriveTrain driveTrain, Vision vision, Arms arms) {
    m_driveTrain = driveTrain;
    m_vision = vision;
    m_arms = arms;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    slot = m_arms.getScoringSlot();
    scoringPositon = (slot >= 8 ? 7 : slot);
    m_driveTrain.setBreakMode();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_vision.closeToPerpendicular(allianceColor, scoringPositon) == false)
      m_driveTrain.arcadeDrive(.5, 0);
    else
      m_driveTrain.arcadeDrive(.35, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_vision.isPerpendicular(allianceColor, scoringPositon);
  }
}
