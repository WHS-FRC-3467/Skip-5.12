// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Control.XBoxControllerEE;

public class Pneumactics extends SubsystemBase{
  /** Creates a new Pneumactics. */
  Compressor phCompressor = new Compressor(PneumaticsModuleType.CTREPCM);
  // PowerDistribution pdh = new PowerDistribution(1, ModuleType.kRev);

  
  // private final XBoxControllerEE m_operatorController = new XBoxControllerEE(1);


  double current = phCompressor.getPressure();

  public Pneumactics() {

  }
  
  @Override
  public void periodic() {
    //pdh.clearStickyFaults();
    
    // if(m_operatorController.getDpadDown()){
    //   phCompressor.enableAnalog(35, 40);
    // }
    // else {
    //   phCompressor.enableAnalog(119, 120);
    // }
    phCompressor.enableDigital();

    SmartDashboard.putNumber("Pressure", phCompressor.getPressure());
  }
}