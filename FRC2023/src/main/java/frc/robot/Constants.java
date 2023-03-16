// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

public final class Constants {
    
    public final class CanIDs {
        public static final int LeftFrontDrive = 31;
        public static final int LeftRearDrive = 34;
        public static final int RightFrontDrive = 32;
        public static final int RightRearDrive = 33;

        //TODO change pickup and feeder motors to right IDs and update firmware
        public static final int leftFingerMotor = 13;
        public static final int rightFingerMotor = 12;


        public static final int ArmRotateMotor = 22;
        public static final int ArmLengthMotor = 23;

        public static final int PCM = 10;
    }

    public final class PCMIDs{
        public static final int Claw_Forward = 4;
        public static final int Claw_Reverse = 5;

        public static final int Arm_Length_Brake_On = 6;
        public static final int Arm_Length_Brake_Off = 7;
    }

    //TODO: tune these values
    public final class ArmRotate{
        public static final double deadband = 2.0;
        
        public static final double maxPosition = 80.0;
        public static final double minPosition = 0.0;

        public static final double RotateKp = 0.04;

    }

    //TODO: tune these values
    public final class ArmLength{
        public static final int armLengthLimitSwitch = 0;
        public static final int armLenghtCurrentOverload = 30;
        public static final int armLengthTempOverload = 38;

        public static final double deadband = 2.0;
        
        public static final double maxPosition = 60.0;
        public static final double minPosition = 0.0;
    }

    //TODO: make the grab positions correct
    public final class GrabPositions{
        public static final double cubeLength = 0;
        public static final double cubeHeight = 4;
        public static final double coneUpLength = 0;
        public static final double coneUpHeight = 4;
        public static final double coneOutLength = 0;
        public static final double coneOutHeight = 4;
        public static final double coneInLength = 0;
        public static final double coneInHeight = 4;
    }

    public final class ScorePositions{
        public static final double lowLength = 1;
        public static final double lowHeight = 25;
        public static final double cubeMedLength = 19;
        public static final double cubeMedHeight = 30;
        public static final double cubeHighLength = 67;
        public static final double cubeHighHeight = 39;
        public static final double coneMedLength = 8.5;
        public static final double coneMedHeight = 38;
        public static final double coneHighLength = 52;
        public static final double coneHighHeight = 50;
    }



    private static final double shift = Units.inchesToMeters(22);
    private static final double offset = Units.inchesToMeters(3);
    public static final int proxSensorInID = 0;
    public static final int ArmRotateLimitID = 1;
    public static final double proxSensorTriggerVoltage = 0.8;
    public static final double txdeadband = 1;
    public static final double yPosDeadband = .04;

    public static final double[] AprilTagYcoordinates = {0, 
        Units.inchesToMeters(174.19) + offset, Units.inchesToMeters(108.19) + offset,
        Units.inchesToMeters(41.19) + offset, Units.inchesToMeters(265.74) + offset, 
        Units.inchesToMeters(265.74) + offset, Units.inchesToMeters(174.19) + offset, 
        Units.inchesToMeters(108.19) + offset, Units.inchesToMeters(41.19) + offset
    };

    public static final double[] BluetargetYcoordinates = {0,
        AprilTagYcoordinates[8] - shift, AprilTagYcoordinates[8], AprilTagYcoordinates[8] + shift, 
        AprilTagYcoordinates[7] - shift, AprilTagYcoordinates[7], AprilTagYcoordinates[7] + shift,
        AprilTagYcoordinates[6] - shift, AprilTagYcoordinates[6], AprilTagYcoordinates[6] + shift
    };

    public static final double ramprate = .02;
    public static final double rampratez = .035;
    public final class Drive {
        public static final double deadband = .005;
    }



}