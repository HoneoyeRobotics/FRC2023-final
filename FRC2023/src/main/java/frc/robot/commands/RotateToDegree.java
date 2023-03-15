// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotPrefs;
import frc.robot.subsystems.DriveTrain;

public class RotateToDegree extends CommandBase {
  private DriveTrain m_DriveTrain;
  private double targetHeading;
  private double rotationSpeed;
  private double sign;
  private double yaw;
  private boolean clockwise;

  private int i;

  /** Creates a new RotateToDegree. */
  public RotateToDegree(DriveTrain driveTrain, double targetHeading) {
    m_DriveTrain = driveTrain;
    this.targetHeading = targetHeading;

    addRequirements(m_DriveTrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    i = 0;
    yaw = m_DriveTrain.getYaw();
    clockwise = (targetHeading - yaw <= 0);

    sign = clockwise ? -1 : 1;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    yaw = m_DriveTrain.getYaw();
    double heading = Math.abs(targetHeading - yaw);
    // rotationSpeed = (Math.abs(targetHeading - yaw) > 25) ?
    // RobotPrefs.getRotateRobotSpeed() : (RobotPrefs.getRotateRobotSpeed() / 1.3);

    if (heading < 30) {
      if (heading < 12.5)
        // ultra fine tune speed
        rotationSpeed = (RobotPrefs.getRotateRobotSpeed() / 1.5);
      else
        // medium fine tune speed
        rotationSpeed = (RobotPrefs.getRotateRobotSpeed() / 1.35);
    } else {
      // full speed
      rotationSpeed = RobotPrefs.getRotateRobotSpeed();
    }

    rotationSpeed *= sign;
    m_DriveTrain.arcadeDrive(0, rotationSpeed);
    i++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_DriveTrain.arcadeDrive(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    yaw = m_DriveTrain.getYaw();

    if (i >= 10) {
      i = 0;
    }
    return (clockwise ? (yaw <= targetHeading + .25) : (yaw >= targetHeading - .25));
  }
}
