package frc.robot.subsystems.Drive;
// Copyright (c) FIRST and other WPILib contributors.

import com.ctre.phoenix.motorcontrol.FeedbackDevice;

// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.swervedrivespecialties.swervelib.Mk3SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanConstants;

import static frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {

  // The maximum voltage that will be delivered to the drive motors.
        public static final double MAX_VOLTAGE = 12.0;
        TalonFX m_frontLeftDriveMotor = new TalonFX(CanConstants.FRONT_LEFT_MODULE_DRIVE_MOTOR);
        TalonFX m_frontRightDriveMotor = new TalonFX(CanConstants.FRONT_RIGHT_MODULE_DRIVE_MOTOR);
        TalonFX m_backLeftDriveMotor = new TalonFX(CanConstants.BACK_LEFT_MODULE_DRIVE_MOTOR);
        TalonFX m_backRightDriveMotor = new TalonFX(CanConstants.BACK_RIGHT_MODULE_DRIVE_MOTOR);

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
        public static final double MAX_VELOCITY_METERS_PER_SECOND = 0.01;
        // 6380.0 / 60.0 *
        // SdsModuleConfigurations.MK3_FAST.getDriveReduction() *
        // SdsModuleConfigurations.MK3_FAST.getWheelDiameter() * Math.PI;

        // The maximum angular velocity of the robot in radians per second.
        // This is a measure of how fast the robot can rotate in place.
        // Here we calculate the theoretical maximum angular velocity. You can also
        // replace this with a measured amount.
        public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND
                        / Math.hypot(DriveConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                                        DriveConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0);

        private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
                        // Front left
                        new Translation2d(DriveConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                                        DriveConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
                        // Front right
                        new Translation2d(DriveConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                                        -DriveConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
                        // Back left
                        new Translation2d(-DriveConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                                        DriveConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
                        // Back right
                        new Translation2d(-DriveConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                                        -DriveConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0));

        // By default we use a Pigeon for our gyroscope. But if you use another
        // gyroscope, like a NavX, you can change this.
        // The important thing about how you configure your gyroscope is that rotating
        // the robot counter-clockwise should
        // cause the angle reading to increase until it wraps back over to zero.
        private final TalonSRX m_pigeonTalon = new TalonSRX(CanConstants.DRIVETRAIN_PIGEON_ID);
        private final PigeonIMU m_pigeon = new PigeonIMU(m_pigeonTalon);

        // These are our modules. We initialize them in the constructor.
        private final SwerveModule m_frontLeftModule;
        private final SwerveModule m_frontRightModule;
        private final SwerveModule m_backLeftModule;
        private final SwerveModule m_backRightModule;

        private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

        public DriveSubsystem() {
                ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

                m_frontLeftModule = Mk3SwerveModuleHelper.createFalcon500(
                        // This parameter is optional, but will allow you to see the current state of
                        // the module on the dashboard.
                        tab.getLayout("Front Left Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(0,0),
                        // This can either be STANDARD or FAST depending on your gear configuration
                        Mk3SwerveModuleHelper.GearRatio.FAST,
                        // This is the ID of the drive motor
                        CanConstants.FRONT_LEFT_MODULE_DRIVE_MOTOR,
                        // This is the ID of the steer motor
                        CanConstants.FRONT_LEFT_MODULE_STEER_MOTOR,
                        // This is the ID of the steer encoder
                        CanConstants.FRONT_LEFT_MODULE_STEER_ENCODER,
                        // This is how much the steer encoder is offset from true zero (In our case,
                        // zero is facing straight forward)
                        DriveConstants.FRONT_LEFT_MODULE_STEER_OFFSET
                );

                // We will do the same for the other modules
                m_frontRightModule = Mk3SwerveModuleHelper.createFalcon500(
                        tab.getLayout("Front Right Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(2,0),
                        Mk3SwerveModuleHelper.GearRatio.FAST, 
                        CanConstants.FRONT_RIGHT_MODULE_DRIVE_MOTOR,
                        CanConstants.FRONT_RIGHT_MODULE_STEER_MOTOR,
                        CanConstants.FRONT_RIGHT_MODULE_STEER_ENCODER,
                        DriveConstants.FRONT_RIGHT_MODULE_STEER_OFFSET
                );

                m_backLeftModule = Mk3SwerveModuleHelper.createFalcon500(
                        tab.getLayout("Back Left Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(4,0),
                        Mk3SwerveModuleHelper.GearRatio.FAST, 
                        CanConstants.BACK_LEFT_MODULE_DRIVE_MOTOR,
                        CanConstants.BACK_LEFT_MODULE_STEER_MOTOR,
                        CanConstants.BACK_LEFT_MODULE_STEER_ENCODER,
                        DriveConstants.BACK_LEFT_MODULE_STEER_OFFSET
                );

                m_backRightModule = Mk3SwerveModuleHelper.createFalcon500(
                        tab.getLayout("Back Right Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(6,0),
                        Mk3SwerveModuleHelper.GearRatio.FAST, 
                        CanConstants.BACK_RIGHT_MODULE_DRIVE_MOTOR,
                        CanConstants.BACK_RIGHT_MODULE_STEER_MOTOR,
                        CanConstants.BACK_RIGHT_MODULE_STEER_ENCODER,
                        DriveConstants.BACK_RIGHT_MODULE_STEER_OFFSET
                );
                
                m_frontLeftDriveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
                m_backLeftDriveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
                m_frontRightDriveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
                m_backRightDriveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
                m_frontLeftDriveMotor.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);
                m_frontLeftDriveMotor.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);
                m_frontLeftDriveMotor.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);
                m_frontLeftDriveMotor.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);
                m_frontLeftDriveMotor.setInverted(false);
                m_backLeftDriveMotor.setInverted(false);
                m_frontRightDriveMotor.setInverted(false);
                m_backRightDriveMotor.setInverted(false);

                m_frontLeftCanCoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
                m_backLeftCanCoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
                m_frontRightCanCoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
                m_backRightCanCoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        }

        @Override
        public void periodic() {
                SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
                // normalize wheel speeds
                SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);

                m_frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                                states[0].angle.getRadians());
                m_frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                                states[1].angle.getRadians());
                m_backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                                states[2].angle.getRadians());
                m_backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                                states[3].angle.getRadians());

                SmartDashboard.putNumber("Front Left Encoder", m_frontLeftDriveMotor.getSelectedSensorPosition());                
                SmartDashboard.putNumber("Front Right Encoder", m_frontRightDriveMotor.getSelectedSensorPosition());
                SmartDashboard.putNumber("Back Left Encoder", m_backLeftDriveMotor.getSelectedSensorPosition());
                SmartDashboard.putNumber("Back Right Encoder", m_backRightDriveMotor.getSelectedSensorPosition());
        }

        public void zeroGyroscope() {
                m_pigeon.setFusedHeading(0.0);
        }

        public Rotation2d getGyroscopeRotation() {
                return Rotation2d.fromDegrees(m_pigeon.getFusedHeading());
        }

        public void drive(ChassisSpeeds chassisSpeeds) {
                m_chassisSpeeds = chassisSpeeds;
        }

        public double getAverageEncoder(){
                //returns in 2048/rotation
                return (m_frontLeftDriveMotor.getSelectedSensorPosition() + m_frontRightDriveMotor.getSelectedSensorPosition() + m_backLeftDriveMotor.getSelectedSensorPosition() + m_backRightDriveMotor.getSelectedSensorPosition())/4;
        }


        public double meterToEncoderTicks(double meters){
                return meters * (2048/(SdsModuleConfigurations.MK3_FAST.getDriveReduction() * SdsModuleConfigurations.MK3_FAST.getWheelDiameter() * Math.PI));
        }

        public void setState(double speed, double angle){
                // Example module states
                var frontLeftState = new SwerveModuleState(speed, Rotation2d.fromDegrees(angle));
                var frontRightState = new SwerveModuleState(speed, Rotation2d.fromDegrees(angle));
                var backLeftState = new SwerveModuleState(speed, Rotation2d.fromDegrees(angle));
                var backRightState = new SwerveModuleState(speed, Rotation2d.fromDegrees(angle));

                // Convert to chassis speeds
                m_chassisSpeeds = m_kinematics.toChassisSpeeds(
                frontLeftState, frontRightState, backLeftState, backRightState);
        }
}