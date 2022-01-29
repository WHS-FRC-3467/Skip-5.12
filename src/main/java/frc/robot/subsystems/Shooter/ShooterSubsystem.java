// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  TalonFX m_shooterMotorLeft = new TalonFX(Constants.CanConstants.ShooterLeft);
  TalonFX m_shooterMotorRight = new TalonFX(Constants.CanConstants.ShooterRight);
  BangBangController BangBang;

  
  public ShooterSubsystem() {


     BangBang = new BangBangController();
     m_shooterMotorLeft.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
     m_shooterMotorRight.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

     m_shooterMotorLeft.configFactoryDefault();
     m_shooterMotorRight.configFactoryDefault();

     m_shooterMotorRight.setInverted(true);
     m_shooterMotorRight.follow(m_shooterMotorLeft);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void setSpeed(double speed) {
    
    m_shooterMotorLeft.set(ControlMode.PercentOutput, BangBang.calculate(getEncoderAverage(), speed));
  }
  public double getEncoderAverage() {
    return (m_shooterMotorLeft.getSelectedSensorVelocity() + m_shooterMotorRight.getSelectedSensorVelocity()) / 2;
  }
}
