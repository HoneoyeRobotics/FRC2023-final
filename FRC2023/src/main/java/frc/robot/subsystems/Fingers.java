// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Fingers extends SubsystemBase {
  private VictorSPX leftFingerMotor;
  private VictorSPX rightFingerMotor;
  /** Creates a new Fingers. */
  public Fingers() {
    leftFingerMotor = new VictorSPX(Constants.CanIDs.leftFingerMotor);
    rightFingerMotor = new VictorSPX(Constants.CanIDs.rightFingerMotor);
    leftFingerMotor.setInverted(true);
  }
  
  public void moveFingers(double percentOutput) {
    leftFingerMotor.set(ControlMode.PercentOutput, percentOutput);
    rightFingerMotor.set(ControlMode.PercentOutput, percentOutput);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}