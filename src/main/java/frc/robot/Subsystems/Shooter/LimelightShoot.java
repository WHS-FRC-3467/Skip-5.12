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
import frc.robot.Feedback.Cameras.LimelightSubsystem;
import frc.robot.Subsystems.Tower.TowerSubsystem;

public class LimelightShoot extends CommandBase {
  /** Creates a new LimelightShoot. */

  //Must be followed by Limelight Aim

  TowerSubsystem m_tower;
  Boolean m_end;
  ShooterSubsystem m_shooter;
  LimelightSubsystem m_limelight;
  double m_velocity, deltaY, count, m_time, m_startTime; 
  NetworkTable table;
  NetworkTableEntry ty;

  /**
   * @param shooter Shooter Subsystem
   * @param tower Tower Subsystem
   * @param limelight LimelightSubsystem
   */
  public LimelightShoot(ShooterSubsystem shooter, TowerSubsystem tower, LimelightSubsystem limelight) {
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

    //Turns on 
    LimelightSubsystem.setVisionMode();

    //Initializes network table member variables
    ty = table.getEntry("ty");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Updates limelight network table member variables 
    ty = table.getEntry("ty");
    deltaY = ty.getDouble(0.0);

    //Zero is tarmac velocity
    m_velocity = deltaY + ShooterConstants.kTarmacVelocity;

    //Updates time
    m_time = Timer.getFPGATimestamp() - m_startTime;
    //Runs shooter at detemined velocity
    m_shooter.shoot(m_velocity, ShooterConstants.kTarmacGains, Value.kForward);  
    
    //If shooter is up to speed or time is greater than one second tower will shoot
    if(m_shooter.isWheelAtSpeed() || m_time > 1.0){
      m_tower.driveWholeTower(TowerConstants.STANDARD_TOWER_SPEED);
    }
    else{
    // else it will stop tower
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
    //stops tower and shooter 
    m_shooter.stopShooter();
    m_tower.driveWholeTower(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_end;
  }
}
