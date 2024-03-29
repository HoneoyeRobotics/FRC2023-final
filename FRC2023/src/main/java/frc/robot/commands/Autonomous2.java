// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Arms;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Fingers;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Autonomous2 extends SequentialCommandGroup {
  private Arms arms;
  private DriveTrain drivetrain;

  private Fingers fingers;
  /** Creates a new Autonomous2. */
  public Autonomous2(Arms arms, DriveTrain drivetrain, Fingers fingers) {
    this.arms = arms;
    this.drivetrain = drivetrain;
    this.fingers = fingers;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        // new ArmMoveIn(arms).withTimeout(.2),
        // new ClawClose(arms),
        // new ScorePiece1(arms),
        // new ScorePiece2(arms),
        // new ClawOpen(arms),
        // new DriveForTime(drivetrain, -.5).withTimeout(.5),
        // new ArmHome(arms),
        
        new FingersOut(fingers).withTimeout(2),
        new DriveUntilTipping(drivetrain, false),
        new BalanceOnPlatform(drivetrain, false));
  }
}
