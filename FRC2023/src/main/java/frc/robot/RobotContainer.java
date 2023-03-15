// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.*;
import frc.robot.enums.*;
import frc.robot.subsystems.*;

public class RobotContainer {
  private Arms arms;
  private DriveTrain driveTrain;
  private Fingers fingers;
  private Vision vision;

  private SendableChooser<Command> selectedAutoMode = new SendableChooser<>();
  private boolean isBlue;

  private CommandXboxController driverJoystick = new CommandXboxController(0);
  private CommandJoystick buttonBoard = new CommandJoystick(1);

  public RobotContainer() {
    driveTrain = new DriveTrain();
    arms = new Arms();
    vision = new Vision();
    fingers = new Fingers();
    // default Teleop command
    driveTrain.setDefaultCommand(new TeleopDrive(driveTrain,
        () -> driverJoystick.getLeftTriggerAxis(),
        () -> driverJoystick.getRightTriggerAxis(),
        () -> driverJoystick.getLeftX()));

    configureBindings();
    configureButtonBoard();
    initializeScorePosition();
    SmartDashboard.putData("ResetNavX", new ResetNavX(driveTrain));
    SmartDashboard.putData("ResetLengthEncoder", new ResetArmLengthEncoder(arms));
    SmartDashboard.putData("ResetRotateEncoder", new ResetArmRotateEncoder(arms));

    selectedAutoMode.setDefaultOption("Score and backup", new Autonomous1(arms, driveTrain));
    selectedAutoMode.addOption("Balance", new Autonomous2(arms, driveTrain));
    SmartDashboard.putData("AutoMode", selectedAutoMode);
  }

  private void configureBindings() {
    // driverJoystick.a().onTrue(new DriveUntilPerpendicular(driveTrain, vision,
    // arms));
    // driverJoystick.b().onTrue(new RotateToDegree(driveTrain, 10));
    driverJoystick.rightBumper().onTrue(new DriveToggleIdleMode(driveTrain));
    driverJoystick.start().onTrue(new MoveToScorePos(driveTrain, vision, arms));

    driverJoystick.back().onTrue(new BombScoreOn(driveTrain));

    driverJoystick.leftBumper().onTrue(new DriveToggleFast(driveTrain));

    driverJoystick.a().whileTrue(new FingersIn(fingers));
    driverJoystick.b().whileTrue(new FingersOut(fingers));

    driverJoystick.y().whileTrue(new BalanceOnPlatform(driveTrain, true));

    //left or right to rotate to peg
    driverJoystick.povLeft().whileTrue(new RotateToPeg(vision, driveTrain));
    driverJoystick.povRight().whileTrue(new RotateToPeg(vision, driveTrain));
    
    //hold up to drive until correct distance
    driverJoystick.povUp().whileTrue(new DriveUntilCorrectDistance(driveTrain, vision));
    //driverJoystick.povUp()
  }

  private void configureButtonBoard() {

    // buttonBoard.button(2).whileTrue(new RunBottomPickup(pickup));
    buttonBoard.button(8).whileTrue(new FingersIn(fingers));

    // buttonBoard.button(6).onTrue(new GrabPositionCycle(arms));
    // buttonBoard.button(7).onTrue(new GrabPiece(arms));
    buttonBoard.button(7).onTrue(new ArmMoveIn(arms).withTimeout(.3).andThen(new ClawClose(arms)));

    buttonBoard.button(4).onTrue(new ClawToggle(arms));

    buttonBoard.button(9).whileTrue(new ArmMoveOut(arms));
    buttonBoard.button(3).whileTrue(new ArmMoveIn(arms));

    buttonBoard.button(10).whileTrue(new ArmRotate(arms, true));
    buttonBoard.button(11).whileTrue(new ArmRotate(arms, false));

    // Brings the arm home
    buttonBoard.button(5).onTrue(
        new ClawClose(arms)
            .andThen(new ArmMoveCompletelyIn(arms))
            .andThen(new ArmRotateToPIDPosition(arms, 0).withTimeout(6))
            .andThen(new ClawOpen(arms)));

    // Changes the scoring position grid on dashboard
    buttonBoard.axisGreaterThan(1, .5).onTrue(new ChangeScoringHeight(arms, false));
    buttonBoard.axisLessThan(1, -.5).onTrue(new ChangeScoringHeight(arms, true));

    buttonBoard.axisGreaterThan(0, .5).onTrue(new ChangeScoringSlot(arms, true));
    buttonBoard.axisLessThan(0, -.5).onTrue(new ChangeScoringSlot(arms, false));

    buttonBoard.button(2).onTrue(new ScorePiece1(arms).andThen(new WaitCommand(.5)).andThen(new ScorePiece2(arms)));
  }

  private void initializeScorePosition() {
    ShuffleboardTab tab2 = Shuffleboard.getTab("Tab 2");
    int i;
    String l_position;
    for (i = 1; i < 10; ++i) {
      l_position = String.format("ScorePos%d%s", i, ScoringHeight.High.toString());
      tab2.add(l_position, false).withPosition(i - 1, 0);
      l_position = String.format("ScorePos%d%s", i, ScoringHeight.Med.toString());
      tab2.add(l_position, false).withPosition(i - 1, 1);
      l_position = String.format("ScorePos%d%s", i, ScoringHeight.Low.toString());
      tab2.add(l_position, false).withPosition(i - 1, 2);
    }
  }

  public Command getAutonomousCommand() {
    return selectedAutoMode.getSelected();
  }
}
