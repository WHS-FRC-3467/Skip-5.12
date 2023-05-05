// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Tower;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Intake.IntakeSubsystem;

public class DriveTower extends CommandBase {
  /** Creates a new DriveTower. */
  TowerSubsystem m_tower;
  DoubleSupplier m_speed;
  /**
   * 
   * @param tower Tower Subsystem
   * @param speed Speed the tower will be run at -1 to 1
   */
  public DriveTower(TowerSubsystem tower, DoubleSupplier speed) { 
    m_speed = speed;
    m_tower = tower;
    addRequirements(m_tower);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if(IntakeSubsystem.getRunning()){
      m_tower.sendToTopWithIntake();
    }
    else{
      m_tower.sendToTopNoIntake();
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_tower.driveWholeTower(0.0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
