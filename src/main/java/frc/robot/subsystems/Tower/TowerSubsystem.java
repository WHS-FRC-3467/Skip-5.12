// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Tower;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanConstants;

public class TowerSubsystem extends SubsystemBase {
  /** Creates a new TowerSubsystem. */
  TalonSRX m_lowerTower = new TalonSRX(CanConstants.LowerTowerMotor);
  TalonSRX m_upperTower = new TalonSRX(CanConstants.UpperTowerMotor);
  public TowerSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
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
}
