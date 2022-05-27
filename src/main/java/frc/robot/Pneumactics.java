// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pneumactics extends SubsystemBase{
  /** Creates a new Pneumactics. */
  Compressor phCompressor = new Compressor(PneumaticsModuleType.REVPH);
  PowerDistribution pdh = new PowerDistribution(1, ModuleType.kRev);

  PneumaticHub m_hub = new PneumaticHub();

  boolean enabled = phCompressor.enabled();
  boolean pressureSwitch = phCompressor.getPressureSwitchValue();
  double current = phCompressor.getPressure();

  public Pneumactics() {

  }
  
  @Override
  public void periodic() {
    pdh.clearStickyFaults();
    phCompressor.enableAnalog(119, 120);
    
    SmartDashboard.putNumber("Pressure", phCompressor.getPressure());
  }
}
