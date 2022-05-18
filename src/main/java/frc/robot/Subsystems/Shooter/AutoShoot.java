// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.TowerConstants;
import frc.robot.Subsystems.Tower.TowerSubsystem;
import frc.robot.Util.Gains;

public class AutoShoot extends CommandBase {
  ShooterSubsystem m_shooter;
  TowerSubsystem m_tower;
  double m_velocity, m_time, m_startTime;
  Gains m_gains;
  Value m_hoodPosition;
  Boolean m_end;
  /**
   * @param shooter Shooter Subsystem
   * @param tower Tower Subsystem
   * @param velocity The velocity of the shooter in RPM
   * @param gains The gains for the shooter
   * @param hoodPosition The hood position in kFoward or kReverse
   */
  public AutoShoot(ShooterSubsystem shooter, TowerSubsystem tower, double velocity, Gains gains, Value hoodPosition) {
    m_shooter = shooter;
    m_tower = tower;
    m_gains = gains;
    m_hoodPosition = hoodPosition;
    m_velocity = velocity;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooter, m_tower);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_time = Timer.getFPGATimestamp() - m_startTime;
    m_shooter.shoot(m_velocity, m_gains, m_hoodPosition);  
    
    if(m_shooter.isWheelAtSpeed() || m_time > 1.0){
      m_tower.driveWholeTower(TowerConstants.standardTowerSpeed);
    }
    else{
      m_tower.driveWholeTower(0.0);
    }

    if(m_tower.ballCount() > 0){
      m_end = false;
    }
    else if(m_tower.ballCount() == 0){
      m_end = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.stopShooter();
    m_tower.driveWholeTower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
