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
        public static final int LowerTowerMotor = 13;
        public static final int UpperTowerMotor = 14;
        public static final int ShooterLeft = 15;
        public static final int ShooterRight = 16;
        public static final int ClimberLeft = 17;
        public static final int ClimberRight = 18;
        public static final int IntakeMotor = 19;
        public static final int DRIVETRAIN_PIGEON_ID = 20; 

        
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
        public static final int FixedClimberForwardSoleniod = 2;
        public static final int FixedClimberReverseSoleniod = 3;
        public static final int ExtendingClimberForwardSoleniod = 4;
        public static final int ExtendingClimberReverseSoleniod = 5;

    }

    public static final class DIOConstants{
        public static final int EntryBeamBreak = 0;
        public static final int MidTowerBeamBreak = 1;
        public static final int UpperTowerBeamBreak = 2;
    }

    public static final class DriveConstants{
    //The left-to-right distance between the drivetrain wheels
    //Should be measured from center to center.

    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.5334; 
    
    //The front-to-back distance between the drivetrain wheels.
    //Should be measured from center to center.
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.5334; 

    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(69.169);
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(105.117);
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(111.181);
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(130.429); 

    public static final double kP = 0.01;
    public static final double kI = 0.0;
    public static final double kD = 0.0008;
    public static final double driveTollerance = 100;

    //speed on -1 to 1 scale
    public static final double SimpleAutoVelocity = 0.25;
    }

    public static final class ShooterConstants {

        public static final double lowerHubVelocity = 2000.0;
	    public static final double upperHubVelocity = 3000.0;

        public static final int kShooterTolerance = 100;
        //double _kP, double _kI, double _kD, double _kF, int _kIzone, double _kPeakOutput
        public static final Gains kGains = new Gains(0.0, 0.0, 0.0, 0.0, 0,  1.00);
            
        public static final double lowerKP = 0.01;
        public static final double lowerKI = 0.0;
        public static final double lowerKD = 1.2;
        public static final double lowerKF = 0.0503;

        public static final double upperKP = 0.01; 
        public static final double upperKI = 0.0;
        public static final double upperKD = 1.15;
        public static final double upperKF = 0.0484;
	}

    public static final class TowerConstants {
        public static final double standardTowerSpeed = 0.75;
    }
    
    public static final class ClimberConstants {
        public static final double kP = 0.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        
        //encoder ticks for 
        public static final double retractedPostion = 0.0;
        //not tuned
        public static final double extendedPosition = 10.0;
		public static final double testSpeed = 0.75;
    }
}
