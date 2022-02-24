// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.TowerConstants;
import frc.robot.subsystems.Tower.TowerSubsystem;

public class ShootUpperHub extends CommandBase {
  ShooterSubsystem m_shooter;
  TowerSubsystem m_tower;
  public ShootUpperHub(ShooterSubsystem shooter, TowerSubsystem tower) {
    m_shooter = shooter;
    m_tower = tower;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooter.shootUpperHub();
    
    if (m_shooter.isWheelAtSpeed()) {
      m_tower.driveWholeTower(TowerConstants.standardTowerSpeed);
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
