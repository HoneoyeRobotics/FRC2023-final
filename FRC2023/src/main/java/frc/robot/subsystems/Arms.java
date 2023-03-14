// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotPrefs;
import frc.robot.Constants.*;
import frc.robot.enums.GrabPosition;
import frc.robot.enums.ScoringHeight;

public class Arms extends SubsystemBase {
  private DigitalInput proxSensorIn;
  private DigitalInput armRotateLimit;
  private boolean armOut;
  private boolean armIn;

  private int scoringSlot = 5;    
  private String l_position;

  private boolean clawOpened;
  private DoubleSolenoid clawSolenoid;
  private boolean armLengthBrakeOn;
  private DoubleSolenoid armLengthBrakeSolenoid;
  private double currentPosition;

  public boolean alreadyClosed = false;

  private CANSparkMax armLengthMotor;
  private CANSparkMax armRotateMotor;

  private boolean armRotatePIDEnabled = true;
  private PIDController armRotatePIDController;
  private double armRotatePIDSetpoint;

  private ScoringHeight scoringHeight = ScoringHeight.Low;
  private GrabPosition grabPosition;
  
  private NetworkTableInstance table = NetworkTableInstance.getDefault();
  private NetworkTable myTable = table.getTable("Shuffleboard/Tab 2");
  //private DigitalInput proxSensor;
  /** Creates a new Arm. */
  public Arms() {
    //TODO: add robot pref for kP value
    armRotatePIDController = new PIDController(RobotPrefs.getArmRotateKp(), 0.0, 0.0);

    armLengthMotor = new CANSparkMax(CanIDs.ArmLengthMotor, MotorType.kBrushless);
    armRotateMotor = new CANSparkMax(CanIDs.ArmRotateMotor, MotorType.kBrushless);
    
    armLengthBrakeSolenoid = new DoubleSolenoid(Constants.CanIDs.PCM, PneumaticsModuleType.CTREPCM, Constants.PCMIDs.Arm_Length_Brake_On, Constants.PCMIDs.Arm_Length_Brake_Off);
    clawSolenoid = new DoubleSolenoid (Constants.CanIDs.PCM, PneumaticsModuleType.CTREPCM, Constants.PCMIDs.Claw_Forward, Constants.PCMIDs.Claw_Reverse);  


    proxSensorIn = new DigitalInput(Constants.proxSensorInID);

    armRotateMotor.getEncoder().setPosition(0.0);
    armRotatePIDController.setSetpoint(0.0);

    armLengthMotor.setIdleMode(IdleMode.kBrake); 
    armRotateMotor.setIdleMode(IdleMode.kBrake);
    armRotatePIDController.setTolerance(1);
    armRotateMotor.setSmartCurrentLimit(15);
    armLengthMotor.setSmartCurrentLimit(40);
    armRotateLimit = new DigitalInput(Constants.ArmRotateLimitID);
  }


  public boolean isClawOpened(){
    return clawOpened;
  }

  public void openClaw(){
    clawSolenoid.set(DoubleSolenoid.Value.kReverse);
    clawOpened = true;
  }
  
  public void closeClaw(){
    alreadyClosed = clawOpened == false;

    clawSolenoid.set(DoubleSolenoid.Value.kForward);
    clawOpened = false;
    
  }


  public boolean isArmLengthBrakeOn(){
    return armLengthBrakeOn;
  }

  public void armLengthBrakeOn(){
    armLengthBrakeSolenoid.set(DoubleSolenoid.Value.kForward);
    armLengthBrakeOn = true;
  }

  public void armLengthBrakeOff(){
    armLengthBrakeSolenoid.set(DoubleSolenoid.Value.kReverse);
    armLengthBrakeOn = false;
  }

  public double armLengthMotorCurrentPosition() {
    return armLengthMotor.getEncoder().getPosition();
  }

  public boolean armLengthOverload() {
    if(armLengthMotor.getMotorTemperature() > ArmLength.armLengthTempOverload){
      SmartDashboard.putBoolean("TempOverload", true);
      return true;
    } 
    else
      return false;
  }

  public void moveArmInOut(double speed){
    //If debug is on youwill ignore the max and min values
    if(RobotPrefs.getDebugMode() == false) {
      if((speed < 0) && (armLengthMotor.getEncoder().getPosition() <= ArmLength.minPosition))
        speed = 0;
      if((speed > 0) && (armLengthMotor.getEncoder().getPosition() >= ArmLength.maxPosition)) 
        speed = 0;
    }

    armLengthMotor.set(speed);
  }

  public void moveArmInCompletely(double speed){
    armLengthMotor.set(speed);
  }

  public  void resetArmLengthEncoder() {
    armLengthMotor.getEncoder().setPosition(0);
  }

  public void moveArmToPosition(double position) {
    currentPosition = armLengthMotor.getEncoder().getPosition();
    double speed = 0;
    if(position > currentPosition)
      speed = RobotPrefs.getArmLengthOutSpeed();
    else
      speed = -1 * RobotPrefs.getArmLengthInSpeed();

    if (position > currentPosition - ArmLength.deadband && position < currentPosition + ArmLength.deadband) {
      speed = 0;
    }

    armLengthMotor.set(speed);
  }


  public void rotateArm(double speed) {
    armRotateMotor.set(speed);
  }

  public void toggleArmRotatePID(){
    armRotatePIDEnabled = !armRotatePIDEnabled;
    //if flipping to false, turn off motor
    if(armRotatePIDEnabled == false)
      armRotateMotor.set(0);
    else
    armRotatePIDSetpoint = armRotateMotor.getEncoder().getPosition();
  }

  public boolean isArmRotateIPDEnabled(){
    return armRotatePIDEnabled;
  }
  
  public void moveArmRotatePIDPosition(double position, boolean setPosition){
    if(setPosition)
      armRotatePIDSetpoint = position;
    else
      armRotatePIDSetpoint += position;

    //If debug mode is off this will ensure the PID does not set the position less than 0 or more than the max
    if(RobotPrefs.getDebugMode() == false) {
      if(position > ArmRotate.maxPosition)
        position = ArmRotate.maxPosition;
      if(position < ArmRotate.minPosition)
        position = ArmRotate.minPosition;
    }
  }

  public boolean isArmRotateAtPosition(){
    // if(RobotPrefs.getDebugMode()) {
    //   SmartDashboard.putBoolean("ArmRotateAtSetpoint", armRotatePIDController.atSetpoint());
    //   SmartDashboard.putNumber("ArmRotateSetpoint", armRotatePIDSetpoint);
    // }
    return armRotatePIDController.atSetpoint();
  }

  public  void resetArmRotateEncoder() {
    armRotateMotor.getEncoder().setPosition(0.0);
    armRotatePIDSetpoint = 0.0;
  }

  public double armRotateMotorCurrentPosition() {
    return armRotateMotor.getEncoder().getPosition();
  }


  public void changeGrabPosition() {
    switch(grabPosition){
      case Cube:
        grabPosition = GrabPosition.ConePointIn;
        break;
      case ConePointIn:
        grabPosition = GrabPosition.ConePointUp;
        break;
      case ConePointUp:
        grabPosition = GrabPosition.ConePointOut;
        break;
      case ConePointOut:
        grabPosition = GrabPosition.Cube;
        break;
      default:
        grabPosition = GrabPosition.Cube;
        break;
    }
    SmartDashboard.putString("GrabPosition", grabPosition.toString());
  }

  public GrabPosition getGrabPosition() {
    return grabPosition;
  }

  public void changeScoringHeight(boolean up) {
    switch(scoringHeight) {
      case Low: 
        if (up) 
          scoringHeight = ScoringHeight.Med;
        break;
      case Med:
        if(up) 
          scoringHeight = ScoringHeight.High;
        else 
          scoringHeight = ScoringHeight.Low;
        break;
      case High:
        if(!up) 
          scoringHeight = ScoringHeight.Med;
        break;
      default:
        scoringHeight = ScoringHeight.Med;
        break;
    }
    SmartDashboard.putString("ScoringHeight", scoringHeight.toString());
    smartDashboardScorePosition();
  }

  public ScoringHeight getScoringHeight() {
    return scoringHeight;
  }

  public void changeScoringSlot(boolean right) {
    if(right) {
      if(scoringSlot == 9)
        scoringSlot = 1;
      else 
        ++scoringSlot;
    }
    else {
      if(scoringSlot == 1) 
        scoringSlot = 9; 
      else
        --scoringSlot;
    }
    SmartDashboard.putNumber("Scoring Slot", scoringSlot);
    smartDashboardScorePosition();
  }

  public int getScoringSlot() {
    return scoringSlot;
  }

  public boolean isCone(int scoringSlot) {
    boolean cone;
    switch(scoringSlot) {
      case 1: case 3: case 4: case 6: case 7: case 9:
        cone = true;
        break;
      case 2: case 5: case 8: default:
        cone = false;
        break;
    }
    return cone;
  }


  public void smartDashboardScorePosition() {
    //NetworkTableEntry myEntry = myTable.getEntry("ScorePos1High");
    //myEntry.setBoolean(true);

    if (l_position != null) {
      myTable.getEntry(l_position).setBoolean(false);
    }
    l_position = String.format("ScorePos%d%s", scoringSlot, scoringHeight.toString());
      myTable.getEntry(l_position).setBoolean(true);
  }


  private void armLocationUpdate() {
    armIn = !proxSensorIn.get();
  }

  public boolean isArmIn() {
    return armIn;
  }

  public boolean isArmOut() {
    return armLengthMotorCurrentPosition() > ArmLength.maxPosition;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    if(armRotatePIDEnabled){
      
      double rotateSpeed = armRotatePIDController.calculate(armRotateMotorCurrentPosition(), armRotatePIDSetpoint);
      if(rotateSpeed < 0 && armRotateLimit.get() == false)
      {
        rotateSpeed = 0;
        resetArmRotateEncoder();
        armRotatePIDSetpoint = armRotateMotorCurrentPosition();
      }
      else if (rotateSpeed < 0)
      {
        rotateSpeed = rotateSpeed / 4;
      }
      armRotateMotor.set(rotateSpeed);
      SmartDashboard.putNumber("armRotateMotorCurrentPosition", armRotateMotorCurrentPosition());
      SmartDashboard.putNumber("armRotatePIDSetpoint", armRotatePIDSetpoint);
      SmartDashboard.putNumber("ArmRotatePIDSpeed", rotateSpeed);
    }
    armLocationUpdate();
    SmartDashboard.putBoolean("armIn", armIn);
    SmartDashboard.putBoolean("armOut", armOut);
    SmartDashboard.putBoolean("armDown", armRotateLimit.get() == false);
    SmartDashboard.putNumber("currentDrawRotate", armRotateMotor.getOutputCurrent());
    SmartDashboard.putNumber("currentDrawLength", armLengthMotor.getOutputCurrent());
    SmartDashboard.putNumber("LengthEncoer", armLengthMotorCurrentPosition());
    
    SmartDashboard.putBoolean("atSetpoint", armRotatePIDController.atSetpoint());

    //SmartDashboard.putBoolean("ProxSensor", proxSensor.get());
  }
}
