// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package Team3467.robot.subsystems.DriveSubsystem;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import Team3467.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  
  private final TalonFX m_rightDrive1 = new TalonFX(Constants.CanConstants.kdriveMotorRight1);
  private final TalonFX m_rightDrive2 = new TalonFX(Constants.CanConstants.kdriveMotorRight2);
  private final TalonFX m_leftDrive1 = new TalonFX(Constants.CanConstants.kdriveMotorLeft1);
  private final TalonFX m_leftDrive2 = new TalonFX(Constants.CanConstants.kdriveMotorLeft2);

  public DriveSubsystem() {

    m_leftDrive1.setInverted(true);
    m_leftDrive2.setInverted(true);    

    m_leftDrive1.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    m_leftDrive2.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    m_rightDrive1.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    m_rightDrive2.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double getAvgEncoder(){
    return (m_leftDrive1.getSelectedSensorPosition() + m_rightDrive1.getSelectedSensorPosition() + m_rightDrive2.getSelectedSensorPosition() + m_leftDrive2.getSelectedSensorPosition())/4;
  }
}
