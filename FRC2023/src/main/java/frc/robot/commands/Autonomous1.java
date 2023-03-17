// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Arms;
import frc.robot.subsystems.DriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Autonomous1 extends SequentialCommandGroup {
  private Arms arms;
  private DriveTrain drivetrain;

  /** Creates a new Autonomous1. */
  public Autonomous1(Arms arms, DriveTrain drivetrain) {
    this.arms = arms;
    this.drivetrain = drivetrain;

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ArmMoveIn(arms).withTimeout(.2),
        new ClawClose(arms),
        new WaitCommand(0.5),
        
        new ScorePiece1(arms),
        new ScorePiece2(arms),
        new WaitCommand(1),        
        new DriveForTime(drivetrain, 0.25).withTimeout(0.5),
        new ClawToggle(arms),
        new WaitCommand(0.5),
        new DriveForTime(drivetrain, -.5).withTimeout(.5),
        new ParallelCommandGroup(
          new ArmHome(arms),
          new DriveForTime(drivetrain, -.5).withTimeout(3))
        );
  }
}
