// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.ejml.sparse.csc.linsol.qr.LinearSolverQrLeftLooking_DSCC;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotPrefs;
import frc.robot.Constants.CanIDs;;

public class DriveTrain extends SubsystemBase {

  private boolean bombScore = false;

  private CANSparkMax leftFrontMotor;
  private CANSparkMax leftRearMotor;
  private CANSparkMax rightFrontMotor;
  private CANSparkMax rightRearMotor;

  private MotorControllerGroup leftMotors;
  private MotorControllerGroup rightMotors;
  private DifferentialDrive drive;

  private boolean driveFast = false;

  private AHRS navx;

  /** Creates a new DriveTrain. */
  public DriveTrain() {
    leftFrontMotor = new CANSparkMax(CanIDs.LeftFrontDrive, MotorType.kBrushless);
    leftRearMotor = new CANSparkMax(CanIDs.LeftRearDrive, MotorType.kBrushless);
    rightFrontMotor = new CANSparkMax(CanIDs.RightFrontDrive, MotorType.kBrushless);
    rightRearMotor = new CANSparkMax(CanIDs.RightRearDrive, MotorType.kBrushless);

    leftMotors = new MotorControllerGroup(leftFrontMotor, leftRearMotor);
    rightMotors = new MotorControllerGroup(rightFrontMotor, rightRearMotor);

    drive = new DifferentialDrive(leftMotors, rightMotors);
    setCoastMode();
    resetEncoders();

    navx = new AHRS(SerialPort.Port.kUSB);
    navx.calibrate();
  }

  public void toggleFast() {
    if (driveFast)
      driveFast = false;
    else
      driveFast = true;
  }

  public boolean getFast() {
    return driveFast;
  }

  public void arcadeDrive(double xSpeed, double zRotation) {
    drive.arcadeDrive(xSpeed, zRotation);
  }

  public void resetEncoders() {
    leftFrontMotor.getEncoder().setPosition(0);
    leftRearMotor.getEncoder().setPosition(0);
    rightFrontMotor.getEncoder().setPosition(0);
    rightRearMotor.getEncoder().setPosition(0);
  }

  public void setBreakMode() {
    leftFrontMotor.setIdleMode(IdleMode.kBrake);
    leftRearMotor.setIdleMode(IdleMode.kBrake);
    rightFrontMotor.setIdleMode(IdleMode.kBrake);
    rightRearMotor.setIdleMode(IdleMode.kBrake);
  }

  public void setCoastMode() {
    leftFrontMotor.setIdleMode(IdleMode.kCoast);
    leftRearMotor.setIdleMode(IdleMode.kCoast);
    rightFrontMotor.setIdleMode(IdleMode.kCoast);
    rightRearMotor.setIdleMode(IdleMode.kCoast);
  }

  public boolean getBrakeMode() {
    return leftFrontMotor.getIdleMode() == IdleMode.kBrake;
  }

  public double getRoll() {
    return navx.getRoll();
  }

  public double getAngle() {
    return navx.getAngle() * -1;
  }

  public double getPitch() {
    return navx.getPitch();
  }

  public double getYaw() {
    return navx.getYaw() * -1;
  }

  public void resetNavX() {
    navx.reset();
  }

  double last_world_linear_accel_x;
  double last_world_linear_accel_y;

  final double kCollisionThreshold_DeltaG = 0.5f;

  public boolean collisionDetected(boolean lemsWay) {
    boolean collisionDetected = false;
    if (lemsWay == false) {

      double curr_world_linear_accel_x = navx.getWorldLinearAccelX();
      double currentJerkX = curr_world_linear_accel_x - last_world_linear_accel_x;
      last_world_linear_accel_x = curr_world_linear_accel_x;
      double curr_world_linear_accel_y = navx.getWorldLinearAccelY();
      double currentJerkY = curr_world_linear_accel_y - last_world_linear_accel_y;
      last_world_linear_accel_y = curr_world_linear_accel_y;

      if ((Math.abs(currentJerkX) > kCollisionThreshold_DeltaG)
          || (Math.abs(currentJerkY) > kCollisionThreshold_DeltaG)) {
        collisionDetected = true;
      }
      SmartDashboard.putBoolean("CollisionDetected", collisionDetected);

    } else {
      if (navx.getVelocityX() <= .1 || navx.getVelocityY() <= .1)
        collisionDetected = true;
    }
    return collisionDetected;
  }

  public boolean isBlue() {
    return true;
  }

  public void setBombScore() {
    bombScore = true;
  }

  public void clearBombScore() {
    bombScore = false;
  }

  public boolean getBombScore() {
    return bombScore;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if (RobotPrefs.getEncoderAndNavXDisplayed()) {
      SmartDashboard.putNumber("LF Encoder", leftFrontMotor.getEncoder().getPosition());
      SmartDashboard.putNumber("LR Encoder", leftRearMotor.getEncoder().getPosition());
      SmartDashboard.putNumber("RF Encoder", rightFrontMotor.getEncoder().getPosition());
      SmartDashboard.putNumber("RR Encoder", rightRearMotor.getEncoder().getPosition());
      SmartDashboard.putNumber("Rotation", getAngle());
      SmartDashboard.putNumber("Roll (RL Tip)", getRoll());
      SmartDashboard.putNumber("Pitch (FB Tip)", getPitch());
      SmartDashboard.putNumber("Yaw", getYaw());
    }
    SmartDashboard.putBoolean("BrakeMode", getBrakeMode());
    SmartDashboard.putBoolean("bombScore", getBombScore());

  }

}
