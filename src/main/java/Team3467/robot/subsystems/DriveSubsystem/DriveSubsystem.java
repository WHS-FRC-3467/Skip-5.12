// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package Team3467.robot.subsystems.DriveSubsystem;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import Team3467.robot.Constants;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  
  private final WPI_TalonFX m_rightDrive1 = new WPI_TalonFX(Constants.CanConstants.kdriveMotorRight1);
  private final WPI_TalonFX m_rightDrive2 = new WPI_TalonFX(Constants.CanConstants.kdriveMotorRight2);
  private final WPI_TalonFX m_leftDrive1 = new WPI_TalonFX(Constants.CanConstants.kdriveMotorLeft1);
  private final WPI_TalonFX m_leftDrive2 = new WPI_TalonFX(Constants.CanConstants.kdriveMotorLeft2);

  private final DifferentialDrive m_drive;

  MotorControllerGroup m_rightControllers;
  MotorControllerGroup m_leftControllers;
  
  public DriveSubsystem() {

    m_leftDrive1.setInverted(true);
    m_leftDrive2.setInverted(true);    

    m_rightControllers = new MotorControllerGroup(m_rightDrive1, m_rightDrive2);
    m_leftControllers = new MotorControllerGroup(m_rightDrive1, m_rightDrive2);
    
    m_drive = new DifferentialDrive(m_leftControllers, m_rightControllers);

    m_leftDrive1.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    m_leftDrive2.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    m_rightDrive1.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    m_rightDrive2.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void arcadeDrive(double fwd, double rot)
  { 
    m_drive.arcadeDrive(fwd, rot);
  }

  public double getAvgEncoder(){
    return (m_leftDrive1.getSelectedSensorPosition() + m_rightDrive1.getSelectedSensorPosition() + m_rightDrive2.getSelectedSensorPosition() + m_leftDrive2.getSelectedSensorPosition())/4;
  }
}
