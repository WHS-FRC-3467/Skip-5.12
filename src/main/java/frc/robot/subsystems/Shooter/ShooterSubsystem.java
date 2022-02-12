// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  TalonFX m_shooterMotorLeft = new TalonFX(Constants.CanConstants.ShooterLeft);
  TalonFX m_shooterMotorRight = new TalonFX(Constants.CanConstants.ShooterRight);
  Servo m_hoodActuator = new Servo(Constants.PWMConstants.HoodAcuator);
  
  public ShooterSubsystem() { 
    m_shooterMotorLeft.configFactoryDefault();
    m_shooterMotorRight.configFactoryDefault();
    m_shooterMotorLeft.setInverted(true);

    m_shooterMotorLeft.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 100);
    m_shooterMotorRight.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 100);

    m_shooterMotorLeft.configNominalOutputForward(0);
    m_shooterMotorLeft.configNominalOutputReverse(0);
    m_shooterMotorLeft.configPeakOutputForward(1);
    m_shooterMotorLeft.configPeakOutputReverse(-1);

    m_shooterMotorLeft.config_kP(0 , ShooterConstants.kP);
    m_shooterMotorLeft.config_kI(0 , ShooterConstants.kI);
    m_shooterMotorLeft.config_kD(0 , ShooterConstants.kD);
    m_shooterMotorLeft.config_kF(0 , ShooterConstants.kF);

    m_shooterMotorRight.config_kP(0 , ShooterConstants.kP);
    m_shooterMotorRight.config_kI(0 , ShooterConstants.kI);
    m_shooterMotorRight.config_kD(0 , ShooterConstants.kD);
    m_shooterMotorRight.config_kF(0 , ShooterConstants.kF);
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("ShooterVelocity", getEncoderAverage());
  }

  public void setSpeed(double velocity){
    m_shooterMotorLeft.set(ControlMode.Velocity, velocity);
    m_shooterMotorLeft.set(ControlMode.Velocity, velocity);
  }

  public double getEncoderAverage() {
    return (m_shooterMotorLeft.getSelectedSensorVelocity() + m_shooterMotorRight.getSelectedSensorVelocity()) / 2;
  }

  public void setHoodPosition(double pos) {
    m_hoodActuator.setPosition(pos);
  }
}
