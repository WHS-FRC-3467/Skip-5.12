// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Tower.TowerSubsystem;

public class TestShoot extends CommandBase {
  /** Creates a new TestShoot. */
  ShooterSubsystem m_shooter;
  TowerSubsystem m_tower;
  public TestShoot(TowerSubsystem tower, ShooterSubsystem shooter) {
  m_tower = tower;
  m_shooter = shooter;
  addRequirements(m_shooter, m_tower);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooter.testShoot();
    if(m_shooter.isWheelAtSpeed()){
      m_tower.driveWholeTower(0.75);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.stopShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
