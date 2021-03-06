// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Tower;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanConstants;
import frc.robot.Constants.DIOConstants;
import frc.robot.Constants.TowerConstants;

public class TowerSubsystem extends SubsystemBase {

  //Tower Subsystem
  TalonSRX m_lowerTower = new TalonSRX(CanConstants.LOWER_TOWER_MOTOR);
  TalonSRX m_upperTower = new TalonSRX(CanConstants.UpperTowerMotor); 
  DigitalInput m_entryBeamBreak = new DigitalInput(DIOConstants.EntryBeamBreak);
  DigitalInput m_midBeamBreak = new DigitalInput(DIOConstants.MidTowerBeamBreak);
  DigitalInput m_upperBeamBreak = new DigitalInput(DIOConstants.UpperTowerBeamBreak);

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
    m_lowerTower.set(ControlMode.PercentOutput, 0.75);
  }
  
  public void fullUpperTower(){
    m_upperTower.set(ControlMode.PercentOutput, 0.75);
  }
  public void sendToTop(){
    if(m_entryBeamBreak.get() == true && m_midBeamBreak.get() == true && m_upperBeamBreak.get() == true){
      //no balls
      driveWholeTower(0.0);
    }
    else if(m_entryBeamBreak.get()==false && m_midBeamBreak.get()==true && m_upperBeamBreak.get()==true){
      //Ball entry
      driveWholeTower(TowerConstants.STANDARD_TOWER_SPEED);
    }
    else if(m_entryBeamBreak.get()==true && m_midBeamBreak.get()== false && m_upperBeamBreak.get() == true){
      //Ball middle
      driveWholeTower(TowerConstants.STANDARD_TOWER_SPEED);
    }
    else if(m_entryBeamBreak.get() == true && m_midBeamBreak.get() == true && m_upperBeamBreak.get() == false){
      //ball upper
      driveWholeTower(0.0);
    }
    else if(m_entryBeamBreak.get()==true && m_midBeamBreak.get() == false && m_upperBeamBreak.get() == false){
      //ball mid and upper
      driveWholeTower(0.0);

    }
    else if(m_entryBeamBreak.get() == false && m_midBeamBreak.get() == true && m_upperBeamBreak.get() == false){
      //ball entry and upper
      driveWholeTower(0.0);
    }
    
    else if(m_entryBeamBreak.get() == false && m_midBeamBreak.get() == false && m_upperBeamBreak.get() == true){
      //ball entry and middle
      driveLowerTower(TowerConstants.STANDARD_TOWER_SPEED);
    }
    else if(m_entryBeamBreak.get() == true && m_midBeamBreak.get() == true && m_upperBeamBreak.get() == true){
      //no balls
      driveWholeTower(0.0);
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
