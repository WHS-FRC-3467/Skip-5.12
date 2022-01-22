// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class CanConstants{
        //drivebase CAN IDs 
        public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 1;
        public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 2;
        public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 3; 
        public static final int BACK_LEFT_MODULE_STEER_MOTOR = 4;
        public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 5; 
        public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 6; 
        public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 7;
        public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 8; 
        public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 9; 
        public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 10; 
        public static final int BACK_LEFT_MODULE_STEER_ENCODER = 11; 
        public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 12; 
        public static final int DRIVETRAIN_PIGEON_ID = 13; 
        public static final int LowerTowerMotor = 13;
        public static final int UpperTowerMotor = 14;
        public static final int ShooterLeft = 15;
        public static final int ShooterRight = 16;
        public static final int ClimberLeft = 17;
        public static final int ClimberRight = 18;
        public static final int IntakeMotor = 19;
    }

    public static final class PWMConstants{
        public static final int HoodAcuator = 0;
        public static final int Blinkin1 = 1;
        public static final int Blinkin2 = 2;
    }
    public static final class PHConstants{
        //PH is pneumatic Hub new PCM
        public static final int IntakeForwardSoleniod = 0;
        public static final int IntakeReverseSoleniod = 1;
        public static final int ClimberForwardSoleniod = 2;
        public static final int ClimberReverseSoleniod = 3;
    }
    public static final class DIOConstants{
        public static final int EntryBeamBreak = 0;
        public static final int MidTowerBeamBreak = 1;
        public static final int UpperTowerBeamBreak = 2;
    }
    public static final class DriveConstants{

    //The left-to-right distance between the drivetrain wheels
    //Should be measured from center to center.
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 1.0; 
    
    //The front-to-back distance between the drivetrain wheels.
    //Should be measured from center to center.
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.5334; 

    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(104.179);
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(115.75);
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(324.755);
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(108.896); 

    public static final double kP = 0.01;
    public static final double kI = 0.0;
    public static final double kD = 0.0008;

    public static final double driveTollerance = 100;
    }
}