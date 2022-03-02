// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pneumactics extends SubsystemBase{
  /** Creates a new Pneumactics. */
  Compressor phCompressor = new Compressor(1, PneumaticsModuleType.REVPH);
  PneumaticHub m_hub = new PneumaticHub();

  boolean enabled = phCompressor.enabled();
  boolean pressureSwitch = phCompressor.getPressureSwitchValue();
  double current = phCompressor.getPressure();

  public Pneumactics() {

  }
  
  @Override
  public void periodic() {
    phCompressor.enableAnalog(115, 120);
    SmartDashboard.putNumber("Pressure", phCompressor.getPressure());
    // This method will be called once per scheduler run
  }
}
