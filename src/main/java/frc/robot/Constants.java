// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.robot.Util.Gains;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final boolean tuningMode = false;

    public static final class CanConstants{
        //drivebase CAN IDs 
        public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 22;
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
        
		//non drivebase CAN IDs
        public static final int LOWER_TOWER_MOTOR = 13;
        public static final int UpperTowerMotor = 14;
        public static final int ShooterLeft = 15; 
        public static final int ShooterRight = 16;
        public static final int ClimberLeft = 17;
        public static final int ClimberRight = 18;
        public static final int IntakeMotor = 19;
        public static final int DRIVETRAIN_PIGEON_ID = 20; 
    }

    public static final class PWMConstants{
        public static final int Blinkin1 = 0;
        public static final int Blinkin2 = 4;
    }
    
    public static final class PHConstants{

        public static final int IntakeForwardSolenoid = 0;
        public static final int IntakeReverseSolenoid = 1;
        public static final int FixedClimberVerticalSolenoid = 2;
        public static final int FixedClimberAngledSolenoid = 3;
        public static final int ExtendingClimberAngledSolenoid = 4;
        public static final int ExtendingClimberVerticalSolenoid = 5;
        public static final int HoodForwardSolenoid = 6;
        public static final int HoodReverseSolenoid = 7;
    }
    
	public static final class DIOConstants{
        public static final int EntryBeamBreak = 0;
        public static final int MidTowerBeamBreak = 1;
        public static final int UpperTowerBeamBreak = 2;
    }
	
    public static final class RobotConstants {
        //
        // Robot physical dimensions and mass quantities.
        //

        // The left-to-right distance between the drivetrain wheels
        // * Should be measured from center to center.
        public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.5334; // 21 inches
        
        // The front-to-back distance between the drivetrain wheels.
        // * Should be measured from center to center.
        public static final double DRIVETRAIN_WHEELBASE_METERS = 0.5334;  // 21 inches
    }

    public static final class DriveConstants{
        public static final double kDeadBand = 0.2;
        public static final boolean PRACTICE = true;
    
        public static final SwerveDriveKinematics DRIVETRAIN_KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(RobotConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, RobotConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
            new Translation2d(RobotConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -RobotConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
            new Translation2d(-RobotConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, RobotConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
            new Translation2d(-RobotConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -RobotConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0)
        );

        public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(202.5);
        public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(103.7);
        public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(144.22); 
        public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(131.1);

        // Limelight auto aim X-axis target tolerance. This is the number of degrees
        // from perfect center that the robot will consider the BasicLimelightAim
        // command "finished".
        public static final double LIMELIGHT_X_TOLERANCE = 1.0;

        // Maximum Limelight auto-aim rotation velocity in radians per second.
        public static final double LIMELIGHT_X_VELOCITY_LIMIT = 0.5;

        // Limelight auto-aim X-axis P-gain.
        public static final double LIMELIGHT_X_P = 10.0;

            
        public static final double precisionSpeed = 0.25;

        //meters per second
        public static final double SimpleAutoVelocity = 1.0;
    }

    public static final class ShooterConstants {

        public static final double kLowerHubVelocity = 975.0;

        public static final double kUpperHubFenderVelocity = 1960.0;

        public static final double kTarmacVelocity = 2100.0;
        
        public static final double kLaunchpadVelocity = 2500;
        
        public static final int kShooterTolerance = 150;
        
        //double _kP, double _kI, double _kD, double _kF, int _kIzone, double _kPeakOutput
        public static final Gains kTestGains = new Gains(0.01, 0.0, 1.15, 0.048, 0, 1.0);
        
        public static final Gains kLowerHubGains = new Gains(0.01, 0.0, 1.05, 0.059, 0, 1.0);

        public static final Gains kUpperHubFenderGains = new Gains(0.035, 0.0, 0.9, 0.0535, 0, 1.0);

        public static final Gains kTarmacGains = new Gains(0.04, 0.0, 2.0, 0.0535, 0, 1.0);

        public static final Gains kLaunchpadGains =  new Gains(0.03, 0.0, 2.0, 0.0535, 0, 1.0);
	}

    public static final class TowerConstants {
        public static final double STANDARD_TOWER_SPEED = 0.75;
    }
    
    public static final class ClimberConstants {

        /**
         * PID Gains may have to be adjusted based on the responsiveness of control loop.
         * kF: 1023 represents output value to Talon at 100%, 6800 represents Velocity units at 100% output
         * 
         * 	                                    			   kP   kI   kD   kF   Iz   PeakOut */
        public final static Gains kGains_Distance = new Gains( 0.5, 0.0, 0.01, .05, 100, 1.00 );
        public final static Gains kGains_Turning  = new Gains( 0.2, 0.0, 0.0, 0.0, 200, 1.00 );
	
	    /* Motor neutral dead-band : Range 0.001 -> 0.25 */
	    public final static double kNeutralDeadband = 0.001;

	    /* Current Limit for arm calibration */
        public final static double kCalibCurrentLimit = 10.0;

        /**
    	 * Set to zero to skip waiting for confirmation.
	     * Set to nonzero to wait and report to DS if action fails.
	    */
	    public final static int kTimeoutMs = 30;

        // Motion Magic constants
        public static final int kMotionCruiseVelocity = 25000;
        public static final int kMotionAcceleration = 35000;
        public static final int kSlowMotionAccel = 19000;
        public final static int kCurveSmoothing = 0;  /* Valid values: 0 -> 8 */
        public static final int kTolerance = 500;

        // Setpoints (in encoder ticks) (not tuned)
        public static final double kClimbingRetractedPostion = 1000.0;
        public static final double kRestingRetractedPostion = 4000.0;
        public static final double kExtendedAboveBar = 50000.0;
        public static final double kFixedArmsFree = 70000.0;
        public static final double kFullExtendedPosition = 205000.0;
    }
}
