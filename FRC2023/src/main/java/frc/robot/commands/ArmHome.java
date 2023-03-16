// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Arms;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ArmHome extends SequentialCommandGroup {
  private Arms arms;

  /** Creates a new ArmHome. */
  public ArmHome(Arms arms) {
    this.arms = arms;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ClawClose(arms),
        new ArmMoveCompletelyIn(arms),
        new ArmRotateToPIDPosition(arms, 5),
        new WaitCommand(.25),
        new ArmRotateToPIDPosition(arms, 1.5),
        new WaitCommand(.25),
        new ArmRotateToPIDPosition(arms, 0),
        new ClawOpen(arms));
  }
}
