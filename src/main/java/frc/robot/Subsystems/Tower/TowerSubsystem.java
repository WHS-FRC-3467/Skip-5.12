// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Tower;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanConstants;
import frc.robot.Constants.DIOConstants;

public class TowerSubsystem extends SubsystemBase {

  //Tower Subsystem
  TalonSRX m_lowerTower = new TalonSRX(CanConstants.LOWER_TOWER_MOTOR);
  TalonSRX m_upperTower = new TalonSRX(CanConstants.UpperTowerMotor); 
  DigitalInput m_entryBeamBreak = new DigitalInput(DIOConstants.EntryBeamBreak);
  DigitalInput m_midBeamBreak = new DigitalInput(DIOConstants.MidTowerBeamBreak);
  DigitalInput m_upperBeamBreak = new DigitalInput(DIOConstants.UpperTowerBeamBreak);
  ColorSensorV3 m_colorSensor = new ColorSensorV3(Port.kOnboard);

  public TowerSubsystem(){
    m_lowerTower.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 255);
    m_lowerTower.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 255);
    m_lowerTower.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 255);
    m_lowerTower.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 255);
    m_lowerTower.setStatusFramePeriod(StatusFrame.Status_17_Targets1, 255);
    m_lowerTower.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 255);

    m_upperTower.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 255);
    m_upperTower.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 255);
    m_upperTower.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 255);
    m_upperTower.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 255);
    m_upperTower.setStatusFramePeriod(StatusFrame.Status_17_Targets1, 255);
    m_upperTower.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 255);
  }

  public boolean m_entryState, m_middleState, m_upperState, m_middleTopState, m_entryTopState, m_entryMiddleState, m_noBallState;
  
  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Entry Beam Break", m_entryBeamBreak.get());
    SmartDashboard.putBoolean("Mid Beam Break", m_midBeamBreak.get());
    SmartDashboard.putBoolean("Upper Beam Break", m_upperBeamBreak.get());
    SmartDashboard.putNumber("Intake CS Prox", m_colorSensor.getProximity());
  }

  /**
   * @param speed the speed the lower tower will be run at -1 to 1
   */
  public void driveLowerTower(double speed){
    m_lowerTower.set(ControlMode.PercentOutput, speed);
  }
  /**
   * @param speed the speed the upper tower will be run at -1 to 1
   */
  public void driveUpperTower(double speed){
    m_upperTower.set(ControlMode.PercentOutput, speed);
  }
  /**
   * @param speed the speed the whole tower will be run at -1 to 1
   */
  public void driveWholeTower(double speed){
    m_upperTower.set(ControlMode.PercentOutput, speed);
    m_lowerTower.set(ControlMode.PercentOutput, speed);
  }

  public void fullDriveTower(){
    m_upperTower.set(ControlMode.PercentOutput, 1);
    m_lowerTower.set(ControlMode.PercentOutput, 1);
  }
  
  public void fullLowerTower(){
    m_lowerTower.set(ControlMode.PercentOutput, 0.5);
  }
  
  public void fullUpperTower(){
    m_upperTower.set(ControlMode.PercentOutput, 0.5);
  }
  public boolean intakeBall(){
    return m_colorSensor.getProximity() > 150.0;
  }
  public Color getIntakeBallColor(){
    return m_colorSensor.getColor();
  }
  public boolean upperBall(){
    if(m_upperBeamBreak.get() == false){
      return true;
    }
    else{
      return false;
    }
  }
  public boolean midBall(){
    if(m_midBeamBreak.get() == false){
      return true;
    }
    else{
      return false;
    }
  }

  public boolean lowerBall(){
    if(m_entryBeamBreak.get() == false){
      return true;
    }
    else{
      return false;
    }
  }

  public void sendToTop(){
    //5
    if(intakeBall() == false && lowerBall() == false && midBall() == false && midBall() == false){
      //no balls
      driveWholeTower(0.0);
    }
    //10
    else if(intakeBall() == false && lowerBall() == true && midBall() == true && upperBall() == true){
      //ball entry and upper
      driveWholeTower(0.0);
    }
    //2
    else if(intakeBall() == false && lowerBall() == true && midBall() == false && midBall() == false){
      //Ball entry
      driveLowerTower(0.75);
    }
    //3
    else if(intakeBall() == false && lowerBall() == false && midBall() == true && upperBall() == false){
      //Ball middle
      driveWholeTower(0.75);
    }
    //4
    else if(intakeBall() == false && lowerBall() == false && midBall() == false && upperBall() == true){
      //ball upper
      driveWholeTower(0.0);
    }
    //8
    else if(intakeBall() == false && lowerBall() == false && midBall() == true && upperBall() == true){
      //ball mid and upper
      driveWholeTower(0.0);
    }
    //12
    else if(intakeBall() == false && lowerBall() == true && midBall() == false && upperBall() == true){
      //ball entry and upper
      driveWholeTower(0.0);
    }
    //7
    else if(intakeBall() == false && lowerBall() == true && midBall() == true && upperBall() == false){
      //ball entry and middle
      driveWholeTower(0.0);
    }


    //intake balls
    //9
    else if(intakeBall() == true && lowerBall() == true && midBall() == true && upperBall() == false){
      //ball entry and middle and intake
      driveWholeTower(0.0);
    }
    //13
    else if(intakeBall() == true && lowerBall() == false && midBall() == true && upperBall() == true){
      //ball mid and upper and intake
      driveWholeTower(0.0);
    }
    //14
    else if(intakeBall() == true && lowerBall() == true && midBall() == false && upperBall() == true){
      //ball mid and upper and intake
      driveWholeTower(0.0);
    }
    //16
    else if(intakeBall() == true && lowerBall() == true && midBall() == true && upperBall() == true){
      //ball entry and mid and upper and intake
      driveWholeTower(0.0);
    }
    //6
    else if(intakeBall() == true && lowerBall() == true && midBall() == false && upperBall() == false){
      //ball intake and lower
      driveWholeTower(0.75);
    }
    //11
    else if(intakeBall() == true && lowerBall() == false && midBall() == true && upperBall() == false){
      //ball intake and mid
      driveWholeTower(0.75);
    }
    //15
    else if(intakeBall() == true && lowerBall() == false && midBall() == false && upperBall() == true){
      //ball intake and upper
      driveLowerTower(0.75);
    }
    //1
    else if(intakeBall() == true && lowerBall() == false && midBall() == false && upperBall() == false){
      //ball intake
      driveLowerTower(0.75);
    }
  }

  /**
   * 
   * @return the number of balls currently in the robot 0 to 2
   */
  public int ballCount(){
    if(m_entryBeamBreak.get()==false && m_midBeamBreak.get()==true && m_upperBeamBreak.get()==true){
      //Ball entry
      return 1;
    }
    else if(m_entryBeamBreak.get()==true && m_midBeamBreak.get()== false && m_upperBeamBreak.get() == true){
      //Ball middle
      return 1;
    }
    else if(m_entryBeamBreak.get() == true && m_midBeamBreak.get() == true && m_upperBeamBreak.get() == false){
      //ball upper
      return 1;
    }
    else if(m_entryBeamBreak.get()==true && m_midBeamBreak.get() == false && m_upperBeamBreak.get() == false){
      //ball mid and upper
      return 2;
    }
    else if(m_entryBeamBreak.get() == false && m_midBeamBreak.get() == true && m_upperBeamBreak.get() ==false){
      //ball entry and upper
      return 2;
    }
    else if(m_entryBeamBreak.get() == false && m_midBeamBreak.get() == false && m_upperBeamBreak.get() == true){
      //ball entry and middle
      return 2;
    }
    else if(m_entryBeamBreak.get() == true && m_midBeamBreak.get() == true && m_upperBeamBreak.get() == true){
      //no balls
      return 0;
    }
    else{
      return 0;
    }
  }

  /**
   * 
   * @return if a beam break is broken
   */
  public boolean doesBallExist (){
    if(m_entryBeamBreak.get() == true && m_midBeamBreak.get() == true && m_upperBeamBreak.get() == true){
      //no balls
      return true;
    }
    else{
      return false;
    }
  }
}
