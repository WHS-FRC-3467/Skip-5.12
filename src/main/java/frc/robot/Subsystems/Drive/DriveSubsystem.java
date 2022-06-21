package frc.robot.Subsystems.Drive;
// Copyright (c) FIRST and other WPILib contributors.

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanConstants;
import frc.robot.Constants.RobotConstants;

import static frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
        //Creates member variables
        private SwerveDriveOdometry m_odometry;
        private NetworkTableEntry odometryEntry;
        //Initializes feild object
        public final Field2d m_field = new Field2d();

        // The maximum voltage that will be delivered to the drive motors.
        public static final double MAX_VOLTAGE = 12.0;

        //initializes motors and CanCoders
        TalonFX m_frontLeftDriveMotor = new TalonFX(CanConstants.FRONT_LEFT_MODULE_DRIVE_MOTOR);
        TalonFX m_frontRightDriveMotor = new TalonFX(CanConstants.FRONT_RIGHT_MODULE_DRIVE_MOTOR);
        TalonFX m_backLeftDriveMotor = new TalonFX(CanConstants.BACK_LEFT_MODULE_DRIVE_MOTOR);
        TalonFX m_backRightDriveMotor = new TalonFX(CanConstants.BACK_RIGHT_MODULE_DRIVE_MOTOR);

        TalonFX m_frontLeftSteerMotor = new TalonFX(CanConstants.FRONT_LEFT_MODULE_STEER_MOTOR);
        TalonFX m_frontRightSteerMotor = new TalonFX(CanConstants.FRONT_RIGHT_MODULE_STEER_MOTOR);
        TalonFX m_backLeftSteerMotor = new TalonFX(CanConstants.BACK_LEFT_MODULE_STEER_MOTOR);
        TalonFX m_backRightSteerMotor = new TalonFX(CanConstants.BACK_RIGHT_MODULE_STEER_MOTOR);

        CANCoder m_frontLeftCanCoder = new CANCoder(CanConstants.FRONT_LEFT_MODULE_STEER_ENCODER);
        CANCoder m_backLeftCanCoder = new CANCoder(CanConstants.BACK_LEFT_MODULE_STEER_ENCODER);
        CANCoder m_frontRightCanCoder = new CANCoder(CanConstants.FRONT_RIGHT_MODULE_STEER_ENCODER);
        CANCoder m_backRightCanCoder = new CANCoder(CanConstants.BACK_RIGHT_MODULE_STEER_ENCODER);

        // The formula for calculating the theoretical maximum velocity is:
        // <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> *
        // pi
        // By default this value is setup for a Mk3 standard module using Falcon500s to
        // drive.
        // The maximum velocity of the robot in meters per second.
        public static final double MAX_VELOCITY_METERS_PER_SECOND = 5800.0 / 60.0 * SdsModuleConfigurations.MK4_L2.getDriveReduction() * SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI;

        // The maximum angular velocity of the robot in radians per second.
        // This is a measure of how fast the robot can rotate in place.
        // Here we calculate the theoretical maximum angular velocity. You can also
        // replace this with a measured amount.
        public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND
                        / Math.hypot(RobotConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                                RobotConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0);

        //initializes Swerve drive kinematics 
        private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
                        // Front left
                        new Translation2d(RobotConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                                RobotConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
                        // Front right
                        new Translation2d(RobotConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                                        -RobotConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
                        // Back left
                        new Translation2d(-RobotConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                                        RobotConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
                        // Back right
                        new Translation2d(-RobotConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                                        -RobotConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0));

        // By default we use a Pigeon for our gyroscope. But if you use another
        // gyroscope, like a NavX, you can change this.
        // The important thing about how you configure your gyroscope is that rotating
        // the robot counter-clockwise should
        // cause the angle reading to increase until it wraps back over to zero.
        private final Pigeon2 m_pigeon = new Pigeon2(CanConstants.DRIVETRAIN_PIGEON_ID);
        // These are our modules. We initialize them in the constructor.
        private final SwerveModule m_frontLeftModule;
        private final SwerveModule m_frontRightModule;
        private final SwerveModule m_backLeftModule;
        private final SwerveModule m_backRightModule;

        //initializes chasisisSpeeds
        private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

        public DriveSubsystem() {
                //Creates a tab for the drive train
                ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

                //initializes modules
                m_frontLeftModule = Mk4SwerveModuleHelper.createFalcon500(
                        tab.getLayout("Front Left Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(0,0),
                        Mk4SwerveModuleHelper.GearRatio.L2,
                        CanConstants.FRONT_LEFT_MODULE_DRIVE_MOTOR,
                        CanConstants.FRONT_LEFT_MODULE_STEER_MOTOR,
                        CanConstants.FRONT_LEFT_MODULE_STEER_ENCODER,
                        DriveConstants.FRONT_LEFT_MODULE_STEER_OFFSET
                );

                m_frontRightModule = Mk4SwerveModuleHelper.createFalcon500(
                        tab.getLayout("Front Right Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(2,0),
                        Mk4SwerveModuleHelper.GearRatio.L2, 
                        CanConstants.FRONT_RIGHT_MODULE_DRIVE_MOTOR,
                        CanConstants.FRONT_RIGHT_MODULE_STEER_MOTOR,
                        CanConstants.FRONT_RIGHT_MODULE_STEER_ENCODER,
                        DriveConstants.FRONT_RIGHT_MODULE_STEER_OFFSET
                );

                m_backLeftModule = Mk4SwerveModuleHelper.createFalcon500(
                        tab.getLayout("Back Left Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(4,0),
                        Mk4SwerveModuleHelper.GearRatio.L2, 
                        CanConstants.BACK_LEFT_MODULE_DRIVE_MOTOR,
                        CanConstants.BACK_LEFT_MODULE_STEER_MOTOR,
                        CanConstants.BACK_LEFT_MODULE_STEER_ENCODER,
                        DriveConstants.BACK_LEFT_MODULE_STEER_OFFSET
                );

                m_backRightModule = Mk4SwerveModuleHelper.createFalcon500(
                        tab.getLayout("Back Right Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(6,0),
                        Mk4SwerveModuleHelper.GearRatio.L2, 
                        CanConstants.BACK_RIGHT_MODULE_DRIVE_MOTOR,
                        CanConstants.BACK_RIGHT_MODULE_STEER_MOTOR,
                        CanConstants.BACK_RIGHT_MODULE_STEER_ENCODER,
                        DriveConstants.BACK_RIGHT_MODULE_STEER_OFFSET
                );
                
                //Configures sensors and brake mode for motors 
                m_frontLeftDriveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
                m_backLeftDriveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
                m_frontRightDriveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
                m_backRightDriveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

                m_frontLeftDriveMotor.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);
                m_frontRightDriveMotor.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);
                m_backLeftDriveMotor.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);
                m_backRightDriveMotor.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);

                m_frontLeftDriveMotor.setNeutralMode(NeutralMode.Brake);
                m_frontRightDriveMotor.setNeutralMode(NeutralMode.Brake);
                m_backLeftDriveMotor.setNeutralMode(NeutralMode.Brake);
                m_backRightDriveMotor.setNeutralMode(NeutralMode.Brake);

                m_frontLeftSteerMotor.setNeutralMode(NeutralMode.Brake);
                m_frontRightSteerMotor.setNeutralMode(NeutralMode.Brake);
                m_backLeftSteerMotor.setNeutralMode(NeutralMode.Brake);
                m_backRightSteerMotor.setNeutralMode(NeutralMode.Brake);

                //configures drive base cancoders to absolute position
                m_frontLeftCanCoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
                m_backLeftCanCoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
                m_frontRightCanCoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
                m_backRightCanCoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);

                //Initializes odometry
                m_odometry = new SwerveDriveOdometry(m_kinematics, getGyroscopeRotation());
                
                //puts odometry on dashboard
                odometryEntry = tab.add("Odometry", "not found").getEntry();

                //puts field on dashboard
                SmartDashboard.putData("Field", m_field);

                //puts gyro rotation on dashboard
                tab.add("gyro rot", getGyroscopeRotation().getDegrees());
        
        }

        @Override
        public void periodic() {

                //updates odometry network table
                odometryEntry.setString(getCurrentPose().toString());

                //Converts chasis speeds to states
                SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
                
                //Normalize wheel speeds
                SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);
                
                //sets modules to the states
                m_frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                                states[0].angle.getRadians());
                m_frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                                states[1].angle.getRadians());
                m_backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                                states[2].angle.getRadians());
                m_backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                                states[3].angle.getRadians());

                //Updates the odometry
                m_odometry.update(getGyroscopeRotation(), states);
                //Updates Field2d
                m_field.setRobotPose(m_odometry.getPoseMeters());
                //Puts feild on dashboard
                SmartDashboard.putData(m_field);
        }

        //Sets the front of the robot to the direction that the robot is facing when command is called
        public void zeroGyroscope() {
                m_pigeon.setYaw(0.0);
        }

        /**
         * @param deg Degrees to set gyro yaw to 
         */
        public void setGyroscope(double deg) {
                m_pigeon.setYaw(deg);
        }
        
        /**
         * @param chassisSpeeds Chasis speeds 
         * Sets chasis speeds passed into the void as the member variable for chasis speeds  
         */
        public void drive(ChassisSpeeds chassisSpeeds) {
                m_chassisSpeeds = chassisSpeeds;
        }

        //gets the number of ticks of the front left drive motor
        public double getAverageEncoder(){
                //returns in 2048/rotation
                return m_frontLeftDriveMotor.getSelectedSensorPosition();
        }

        //Sets internal encoders of driving swerve motor to 0 
        public void resetDriveEncoders(){
                m_backLeftDriveMotor.setSelectedSensorPosition(0.0);
                m_frontLeftDriveMotor.setSelectedSensorPosition(0.0);
                m_backRightDriveMotor.setSelectedSensorPosition(0.0);
                m_frontRightDriveMotor.setSelectedSensorPosition(0.0);
        }
        /**
         * 
         * @param meters
         * @return encoder ticks 
         * Converts meters to encoder ticks 
         */
        public double meterToEncoderTicks(double meters){
                return meters * (2048/(SdsModuleConfigurations.MK4_L2.getDriveReduction() * SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI));
        }
        /**
         * 
         * @param encoderTicks
         * @return meters for robot drive
         * Converts encoder ticks to meters
         */
        public double encoderTicksToMeter(double encoderTicks){
                return encoderTicks /  (2048/(SdsModuleConfigurations.MK4_L2.getDriveReduction() * SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI));
        }
        
        /**
         * @param pose the Pose2D that the odometry will be set to 
         */
        public void resetOdometry(Pose2d pose){
                m_odometry.resetPosition(pose, pose.getRotation());
        }
        /**
         * gets the yaw of robot in rotation2d 
         */
        public Rotation2d getGyroscopeRotation() {
                return Rotation2d.fromDegrees(m_pigeon.getYaw());
        }
        /** 
         * @return pitch of robot
         */
        public double getGyroPitch() {
                return m_pigeon.getPitch();
        }
        
        /**
         * 
         * @return Pose of robot from the odometry
         */
        public Pose2d getCurrentPose(){
                return m_odometry.getPoseMeters();
        }
        /**
         * 
         * @return the kinematics of the swerve drive
         */
        public SwerveDriveKinematics getKinematics(){
                return m_kinematics;
        }
        /**
         * 
         * @param states converts states to chasis speeds
         */
        public void actuateModulesAuto(SwerveModuleState[] states){
                drive(m_kinematics.toChassisSpeeds(states));
        }   
           
        /**
         * @param value The value of the joystick that will be modified
         * @param exponent The power to which the joystick will be raised to 
         * @return The modified value of the joystick
         */
        public double modifyAxis(double value, int exponent){
                double deadValue = MathUtil.applyDeadband(value, DriveConstants.kDeadBand);
                double quarticValue = Math.copySign(Math.pow(deadValue, exponent), deadValue);
                return quarticValue;
        }
}

