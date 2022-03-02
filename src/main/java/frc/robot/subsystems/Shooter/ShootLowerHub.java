// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.TowerConstants;
import frc.robot.subsystems.Tower.TowerSubsystem;

public class ShootLowerHub extends CommandBase {
  ShooterSubsystem m_shooter;
  TowerSubsystem m_tower;
  double m_time, m_startTime;

  public ShootLowerHub(ShooterSubsystem shooter, TowerSubsystem tower) {
    m_shooter = shooter;
    m_tower = tower;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_startTime = Timer.getFPGATimestamp();
    m_shooter.deployHood();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_time = Timer.getFPGATimestamp() - m_startTime;

    m_shooter.shootLowerHub();
    
    if (m_shooter.isWheelAtSpeed() || m_time > 1.0) {
      m_tower.driveWholeTower(TowerConstants.standardTowerSpeed);
    }
    else{
      m_tower.driveWholeTower(0.0);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.runShooter(0);
    m_tower.driveWholeTower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
