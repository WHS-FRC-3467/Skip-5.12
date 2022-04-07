// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

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
        
		//non drivebase CAN IDs
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
        public static final int Blinkin1 = 0;
        public static final int Blinkin2 = 4;
    }
    
    public static final class PHConstants{
        //PH is pneumatic Hub new PCM
        public static final boolean UseREVPH = false;  // true for REV PH, false for CTRE PCM
        //public static final PneumaticsModuleType PMType = PneumaticsModuleType.REVPH;
        public static final PneumaticsModuleType PMType = PneumaticsModuleType.REVPH;
        ;

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
        public static final double kDeadBand = 0.1;
        public static final boolean PRACTICE = true;
    
        public static final SwerveDriveKinematics DRIVETRAIN_KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(RobotConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, RobotConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
            new Translation2d(RobotConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -RobotConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
            new Translation2d(-RobotConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, RobotConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
            new Translation2d(-RobotConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -RobotConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0)
        );

        public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(278.1);
        public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(104.7);
        public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(110.5); 
        public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(132.6);

        // Drivetrain Performance Mechanical limits
        
        // The maximum velocity of the robot in meters per second.
        // The formula for calculating the theoretical maximum velocity is:
        // <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> * PI
        // By default this value is setup for a MK4 module using Falcon500s to drive.
        public static final double MAX_VELOCITY_METERS_PER_SECOND =
            6380.0 /     // Falcon500 free speed (rpm)
            60.0 *       // sec per min
            SdsModuleConfigurations.MK4_L2.getDriveReduction() *
            SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI;

        // The maximum angular velocity of the robot in radians per second.
        // This is a measure of how fast the robot can rotate in place.
        // Here we calculate the theoretical maximum angular velocity. You can also
        // replace this with a measured amount.
        public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND =
            MAX_VELOCITY_METERS_PER_SECOND /
            Math.hypot(RobotConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                        RobotConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0);

        // These values will be passed to the swerve system, and can be tuned here with measured values if you wish.
        static public final double MAX_FWD_REV_SPEED_MPS = MAX_VELOCITY_METERS_PER_SECOND * 0.75;
        static public final double MAX_STRAFE_SPEED_MPS = MAX_VELOCITY_METERS_PER_SECOND * 0.75;
        static public final double MAX_ROTATE_SPEED_RAD_PER_SEC = MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * 0.5;
        static public final double MAX_TRANSLATE_ACCEL_MPS2 = MAX_FWD_REV_SPEED_MPS/0.25; //0-full time of 0.25 second
        static public final double MAX_ROTATE_ACCEL_RAD_PER_SEC_2 = MAX_ROTATE_SPEED_RAD_PER_SEC/0.25; //0-full time of 0.25 second
        public static final double MAX_VOLTAGE = 12.0; // Maximum Voltage sent to the drive motors

        // Sensor-related constants - pulled from datasheets for the sensors and gearboxes
        static public final int ENC_PULSE_PER_REV = 2048; // TalonFX integrated sensor
        static public final int WHEEL_ENC_COUNTS_PER_WHEEL_REV = ENC_PULSE_PER_REV;  //Assume 1-1 gearing for now
        static public final int STEER_ENC_COUNTS_PER_MODULE_REV = 4096; // CANCoder
        static public final double WHEEL_ENC_WHEEL_REVS_PER_COUNT  = 1.0/((double)(WHEEL_ENC_COUNTS_PER_WHEEL_REV));
        static public final double steer_ENC_MODULE_REVS_PER_COUNT = 1.0/((double)(STEER_ENC_COUNTS_PER_MODULE_REV));

	    // PID Drive Constants
        public static final double kP = 0.01;
        public static final double kI = 0.0;
        public static final double kD = 0.0008;
        public static final double driveTollerance = 100;
            
        public static final double precisionSpeed = 0.25;

        //meters per second
        public static final double SimpleAutoVelocity = 1.0;
    }

    public static final class ShooterConstants {
        //measured 950

        public static final double lowerHubVelocity = 975.0;
        //measured 1900
	    public static final double upperHubVelocity = 1800.0;

        public static final double TarmacVelocity = 1700.0;

        public static final int kShooterTolerance = 100;
        //double _kP, double _kI, double _kD, double _kF, int _kIzone, double _kPeakOutput
        public static final Gains kGains = new Gains(0.0, 0.0, 0.0, 0.0, 0,  1.00);
        
        public static final double kP = 0.01;
        public static final double kI = 0.0;
        public static final double kD = 1.15;
        public static final double kF = 0.048;

        public static final double lowerKP = 0.01;
        public static final double lowerKI = 0.0;
        public static final double lowerKD = 1.05;
        public static final double lowerKF = 0.059;

        public static final double upperKP = 0.035; 
        public static final double upperKI = 0.0;
        public static final double upperKD = 1.0;
        public static final double upperKF = 0.0535;
	}

    public static final class TowerConstants {
        public static final double standardTowerSpeed = 0.75;
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
        public final static double kCalibCurrentLimit = 15.0;

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
        public static final double kClimbingRetractedPostion = 750.0;
        public static final double kRestingRetractedPostion = 3000.0;
        public static final double kExtendedAboveBar = 37500.0;
        public static final double kFixedArmsFree = 52500.0;
        public static final double kFullExtendedPosition = 160000.0;

    }

	public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        public static final double TRAJECTORYXkP = 1;
        public static final double TRAJECTORYYkP = 1;
        public static final double THETACONTROLLERkP = 1;

        // Constraint for the motion profilied robot angle controller
        public static final TrapezoidProfile.Constraints THETACONTROLLERCONSTRAINTS =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }
}
