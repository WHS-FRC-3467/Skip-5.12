// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Feedback.LED;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PWMConstants;

public class LED extends SubsystemBase {
  /** Creates a new LED. */
  Spark sponsorBlinkin = new Spark(PWMConstants.Blinkin1);
  //Spark bottomBlinkin = new Spark(PWMConstants.Blinkin2);

  public LED() {  
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void noBallLight(){
    //Solid s
    sponsorBlinkin.set(0.0);
  }
  public void oneBallLight(){
    //Solid Orange
    sponsorBlinkin.set(-0.05);
  }
  public void twoBallLight(){
    //Solid Green
    sponsorBlinkin.set(0.93);
  }

  // public void bottomLights(){
  //   //Moving reds, yellows, and oranges.
  //   bottomBlinkin.set(-0.73);
  // }
}
