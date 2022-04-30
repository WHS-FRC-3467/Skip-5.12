// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Shooter;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TowerConstants;
import frc.robot.Feedback.Cameras.Limelight;
import frc.robot.Subsystems.Tower.TowerSubsystem;

public class LimelightShoot extends CommandBase {
  /** Creates a new LimelightShoot. */

  //Must be followed by Limelight Aim

  TowerSubsystem m_tower;
  Boolean m_end;
  ShooterSubsystem m_shooter;
  Limelight m_limelight;
  double m_velocity, deltaY, count, m_time, m_startTime; 
  NetworkTable table;
  NetworkTableEntry ty;


  public LimelightShoot(ShooterSubsystem shooter, TowerSubsystem tower, Limelight limelight) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = shooter;
    m_limelight = limelight; 
    m_tower = tower; 
    addRequirements(m_shooter, m_tower, m_limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Initialize parent NetworkTable
    table = NetworkTableInstance.getDefault().getTable("limelight");

    // enable limelight LEDs
    // table.getEntry("ledMode").setNumber(3);


    Limelight.setVisionMode();

    ty = table.getEntry("ty");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ty = table.getEntry("ty");
    deltaY = ty.getDouble(0.0);

    // TODO: Determine equation for m_velocity through testing 
    //Zero is tarmac velocity
    m_velocity = deltaY + ShooterConstants.kTarmacVelocity;

    m_time = Timer.getFPGATimestamp() - m_startTime;
    m_shooter.shoot(m_velocity, ShooterConstants.kTarmacGains, Value.kForward);  
    
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
    m_tower.driveWholeTower(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_end;
  }
}
