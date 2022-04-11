// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Feedback.LED;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Tower.TowerSubsystem;

public class LEDDefault extends CommandBase {
  /** Creates a new LEDDefault. */
  LED m_led;
  TowerSubsystem m_tower;
  double startTime;
  public LEDDefault(LED led, TowerSubsystem tower) {
    m_tower = tower;
    m_led = led;
    addRequirements(m_led);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // m_led.bottomLights();
    // if(m_tower.ballCount() == 0){
    //   m_led.noBallLight();
    // }
    // if(m_tower.ballCount() == 1){
    //   m_led.oneBallLight();
    // }
    // if(m_tower.ballCount() == 2){
      m_led.twoBallLight();
    //}
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
