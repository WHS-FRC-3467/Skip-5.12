// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Tower;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanConstants;
import frc.robot.Constants.DIOConstants;
import frc.robot.Constants.TowerConstants;

public class TowerSubsystem extends SubsystemBase {
  /** Creates a new TowerSubsystem. */
  TalonSRX m_lowerTower = new TalonSRX(CanConstants.LowerTowerMotor);
  TalonSRX m_upperTower = new TalonSRX(CanConstants.UpperTowerMotor); 
  DigitalInput m_entryBeamBreak = new DigitalInput(DIOConstants.EntryBeamBreak);
  DigitalInput m_midBeamBreak = new DigitalInput(DIOConstants.MidTowerBeamBreak);
  DigitalInput m_upperBeamBreak = new DigitalInput(DIOConstants.UpperTowerBeamBreak);

  public boolean doesBallExist;
  public boolean m_entryState, m_middleState, m_upperState, m_middleTopState, m_entryTopState, m_entryMiddleState, m_noBallState;
  public boolean entryBall, midBall, upperBall;

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Entry Beam Break", m_entryBeamBreak.get());
    SmartDashboard.putBoolean("Mid Beam Break", m_midBeamBreak.get());
    SmartDashboard.putBoolean("Upper Beam Break", m_upperBeamBreak.get());
  }

  public void driveLowerTower(double speed){
    m_lowerTower.set(ControlMode.PercentOutput, speed);
  }
  public void driveUpperTower(double speed){
    m_upperTower.set(ControlMode.PercentOutput, speed);
  }
  public void driveWholeTower(double speed){
    m_upperTower.set(ControlMode.PercentOutput, speed);
    m_lowerTower.set(ControlMode.PercentOutput, speed);
  }
  
  public void sendToTop(){
    if(m_entryBeamBreak.get()==false && m_midBeamBreak.get()==true && m_upperBeamBreak.get()==true){
      //Ball entry
      driveWholeTower(TowerConstants.standardTowerSpeed);
    }
    else if(m_entryBeamBreak.get()==true && m_midBeamBreak.get()== false && m_upperBeamBreak.get() == true){
      //Ball middle
      driveWholeTower(TowerConstants.standardTowerSpeed);
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
      driveLowerTower(TowerConstants.standardTowerSpeed);
    }
    else if(m_entryBeamBreak.get() == false && m_midBeamBreak.get() == false && m_upperBeamBreak.get() == true){
      //ball entry and middle
      driveWholeTower(TowerConstants.standardTowerSpeed);
    }
    else if(m_entryBeamBreak.get() == true && m_midBeamBreak.get() == true && m_upperBeamBreak.get() == true){
      //no balls
      driveWholeTower(0.0);
    }
  }

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
  
}