// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Drive;
import frc.robot.subsystems.DriveTrain;

public class NewTeleopDrive extends CommandBase {
  private DriveTrain m_drivetrain;
  private DoubleSupplier m_leftTriggerSupplier;
  private DoubleSupplier m_rightTriggerSupplier;
  private DoubleSupplier m_leftStickXSupplier;
  private double xspeed;
  private double zrotation;
  
  /** Creates a new NewTeleopDrive. */
  public NewTeleopDrive(DriveTrain drivetrain, DoubleSupplier leftTriggerSupplier, DoubleSupplier rightTriggerSupplier,
      DoubleSupplier leftStickXSupplier) {
    m_leftTriggerSupplier = leftTriggerSupplier;
    m_rightTriggerSupplier = rightTriggerSupplier;
    m_leftStickXSupplier = leftStickXSupplier;
    m_drivetrain = drivetrain;
    addRequirements(m_drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    xspeed = m_rightTriggerSupplier.getAsDouble() - m_leftTriggerSupplier.getAsDouble();
    zrotation = (m_leftStickXSupplier.getAsDouble());

    
    if ((zrotation < Drive.deadband) && (zrotation > (Drive.deadband * -1)))
      zrotation = 0;
    else {
      zrotation = zrotation * zrotation * (zrotation > 0 ? 1 : -1);
      zrotation = zrotation * .6;
    }

    if ((xspeed < Drive.deadband) && (xspeed > (Drive.deadband * -1)))
      xspeed = 0;
    else {
      xspeed = xspeed * xspeed * (xspeed > 0 ? 1 : -1);
      xspeed = xspeed * .7;
    }

    m_drivetrain.arcadeDrive(xspeed, zrotation);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
