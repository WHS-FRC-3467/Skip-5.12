// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Feedback.LED;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PWMConstants;

public class LED extends SubsystemBase {
  /** Creates a new LED. */
  Spark towerBlinkin = new Spark(PWMConstants.Blinkin1);
  Spark shooterBlinkin = new Spark(PWMConstants.Blinkin1);

  public LED() {  
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void noBallLight(){
    //Solid Purple
    towerBlinkin.set(0.91);
  }
  public void oneBallLight(){
    //Solid Orange
    towerBlinkin.set(0.65);
  }
  public void twoBallLight(){
    //Solid Green
    towerBlinkin.set(0.77);
  }
  public void shooterOffLight(){
    //Solid Red
    shooterBlinkin.set(0.61);
  }
  public void shooterOnLight(){
    //Solid Blue
    shooterBlinkin.set(0.87);
  }
  public void shooterAtSpeed(){
    //Solid Yellow
    shooterBlinkin.set(0.69);
  }
  public void disabledColor(){
    //Moving reds, yellows, and oranges.
    towerBlinkin.set(-0.73);
    shooterBlinkin.set(-0.73);

  }
}
