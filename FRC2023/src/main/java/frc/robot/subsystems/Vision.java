// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.enums.LimeLightState;

public class Vision extends SubsystemBase {

  private NetworkTable side_limelight = NetworkTableInstance.getDefault().getTable("limelight-suitsx");
  // private NetworkTableEntry side_tx = side_limelight.getEntry("tx");
  private NetworkTableEntry side_coordinates = side_limelight.getEntry("botpose_wpiblue");

  private NetworkTable main_limelight = NetworkTableInstance.getDefault().getTable("limelight-suits");
  private NetworkTableEntry main_tx = main_limelight.getEntry("tx");
  private NetworkTableEntry main_tz = main_limelight.getEntry("camerapose_targetspace");

  private NetworkTableEntry aprilTagID = main_limelight.getEntry("tid");
  public boolean isBlue;

  /** Creates a new Vision. */
  public Vision() {
    setFrontLimelightState(LimeLightState.Drive);
    CameraServer.startAutomaticCapture();
  }

  public double getFrontTx() {
    return main_tx.getDouble(0.0);
  }

  // public boolean isPerpendicular(int aprilTagID) {
  // int currentAprilTagID = (int)(m_aprilTagID.getDouble(0));
  // double target = -5;
  // double current = side_tx.getDouble(10);
  // if(currentAprilTagID == aprilTagID &&
  // (current <= Constants.txdeadband + target && current >= (Constants.txdeadband
  // * -1) + target))
  // return true;
  // else
  // return false;
  // }

  public boolean isPerpendicular(int allianceColor, int scoringPos) {
    // int currentAprilTagID = (int)(m_aprilTagID.getDouble(0));

    // Target coordinates based on the chosen scoring position
    double target = Constants.BluetargetYcoordinates[scoringPos];

    // Current coordinates based on apriltag
    double[] currentPos = side_coordinates.getDoubleArray(new double[] {});

    if (currentPos.length > 2) {
      // checks to see if the current Y position is within the deadband of the target
      // Y position
      if ((currentPos[1] <= Constants.yPosDeadband + target && currentPos[1] >= (Constants.yPosDeadband * -1) + target))
        return true;
    }
    return false;
  }

  public boolean closeToPerpendicular(int allianceColor, int scoringPos) {
    // int currentAprilTagID = (int)(m_aprilTagID.getDouble(0));

    // Target coordinates based on the chosen scoring position
    double target = Constants.BluetargetYcoordinates[scoringPos];

    // Current coordinates based on apriltag
    double[] currentPos = side_coordinates.getDoubleArray(new double[] {});
    if (currentPos[1] == 0.0)
      return true;
    return (currentPos[1] <= target + 0.5 && currentPos[1] >= target - 0.5) ? true : false;
  }

  public boolean correctDistance() {
    double target = -1.7;
    double[] coordinates = main_tz.getDoubleArray(new double[] {});
    if (coordinates.length < 3 || coordinates[2] == 0.0)
      return false;
    if (coordinates[2] >= target - .1)
      return true;
    else
      return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (side_coordinates != null) {
      double[] coordinates = side_coordinates.getDoubleArray(new double[] {});
      if (coordinates.length > 2) {
        SmartDashboard.putNumber("xCord", coordinates[0]);
        SmartDashboard.putNumber("yCord", coordinates[1]);
        SmartDashboard.putNumber("zCord", coordinates[2]);
      }
    }
    // if (aprilTagID.getDouble(0) == 6)
    // isBlue = true;
    // else {
    // if(aprilTagID.getDouble(0) == 3)
    // isBlue = false;
    // }

    // SmartDashboard.putBoolean("AtSeven?", isPerpendicular(0, 4));

    SmartDashboard.putBoolean("isblue", isBlue);
    SmartDashboard.putBoolean("CorrectDistance", correctDistance());
  }

  public void setFrontLimelightState(LimeLightState state) {
    switch (state) {
      // case Drive:
      // main_limelight.getEntry("pipeline").setDouble(1);
      // main_limelight.getEntry("camMode").setDouble(1);
      // break;
      default:
      case AprilTag:
        main_limelight.getEntry("pipeline").setDouble(5);
        main_limelight.getEntry("camMode").setDouble(0);
        break;
      // case Reflective:
      // main_limelight.getEntry("pipeline").setDouble(1);
      // main_limelight.getEntry("camMode").setDouble(0);
      // break;
    }
  }
}
