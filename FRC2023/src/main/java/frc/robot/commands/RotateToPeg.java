// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotPrefs;
import frc.robot.enums.LimeLightState;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Vision;

public class RotateToPeg extends CommandBase {
  private Vision vision;
  private DriveTrain driveTrain;

  public RotateToPeg(Vision vision, DriveTrain driveTrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveTrain = driveTrain;
    this.vision = vision;
    addRequirements(driveTrain);

    pidController = new PIDController(0.2, 0, 0);
    pidController.setTolerance(5, 10);

  }

  private double offset;
  private PIDController pidController;

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // set pipeline to get the forward reflective tags
    vision.setFrontLimelightState(LimeLightState.Reflective);

    // get current offset
    offset = RobotPrefs.getReflectiveTagOffset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // move to position
    driveTrain.arcadeDrive(0, pidController.calculate(vision.getFrontTx(), offset));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.arcadeDrive(0, 0);
    vision.setFrontLimelightState(LimeLightState.AprilTag);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pidController.atSetpoint();
  }
}