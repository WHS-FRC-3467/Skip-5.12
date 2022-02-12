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

public class TowerSubsystem extends SubsystemBase {
  /** Creates a new TowerSubsystem. */
  TalonSRX m_lowerTower = new TalonSRX(CanConstants.LowerTowerMotor);
  TalonSRX m_upperTower = new TalonSRX(CanConstants.UpperTowerMotor); 
  DigitalInput m_entryBeamBreak = new DigitalInput(DIOConstants.EntryBeamBreak);
  DigitalInput m_midBeamBreak = new DigitalInput(DIOConstants.MidTowerBeamBreak);
  DigitalInput m_upperBeamBreak = new DigitalInput(DIOConstants.UpperTowerBeamBreak);

  public boolean doesBallExist = m_entryBeamBreak.get() || m_midBeamBreak.get() || m_upperBeamBreak.get();

  @Override
  public void periodic() {
    // sendToTop();
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

  public void sendToTop() {
    if ((m_upperBeamBreak.get() == false) && (doesBallExist == true)) { // when both motors should be run
      driveWholeTower(0.01); // calibrate for something reasonable 
    } else if ((m_upperBeamBreak.get() == true) && (m_entryBeamBreak.get() == true)) { // when just the lower motor should run
      driveLowerTower(0.01); // calibrate for something reasonable
    } else {
      driveWholeTower(0);
    }
  }
  public int ballCount(){
    if (doesBallExist == false) {
      return 0;
    }
    else if ((m_upperBeamBreak.get()) && (m_midBeamBreak.get())){
      return 2;
    }
    else if(m_midBeamBreak.get() && m_entryBeamBreak.get()){
      return 2;
    }
    else if(m_upperBeamBreak.get() && m_entryBeamBreak.get()){
      return 2;
    }
    else if (m_upperBeamBreak.get()){
      return 1;
    }
    else if (m_midBeamBreak.get()){
      return 1;
    }
    else if (m_entryBeamBreak.get()){
      return 1;
    }
    else{
      return 0;
    }
  }
  
}