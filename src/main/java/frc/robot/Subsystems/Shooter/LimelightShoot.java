// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Shooter;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.TowerConstants;
import frc.robot.Feedback.Cameras.LimelightSubsystem;
import frc.robot.Subsystems.Tower.TowerSubsystem;

public class LimelightShoot extends CommandBase {
  /** Creates a new LimelightShoot. */

  //Must be followed by Limelight Aim

  TowerSubsystem m_tower;
  ShooterSubsystem m_shooter;
  double m_velocity, deltaY, deltaX, errorX, errorY; 
  NetworkTable table;
  NetworkTableEntry tx, ty, ta, tv;
  boolean m_waitForTarget, m_hasTarget;


  /**
   * @param shooter Shooter Subsystem
   * @param tower Tower Subsystem
   * @param limelight LimelightSubsystem
   */
  public LimelightShoot(ShooterSubsystem shooter, TowerSubsystem tower) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = shooter;
    m_tower = tower; 
    addRequirements(m_shooter, m_tower);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Initialize parent NetworkTable
    table = NetworkTableInstance.getDefault().getTable("limelight");

    //Turns on 
    LimelightSubsystem.setVisionMode();

    //Initializes network table member variables
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    tv = table.getEntry("tv");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    ty = table.getEntry("ty");
    tv = table.getEntry("tv");
	  //removed ta since it's unused and slows the loop
	
    // get current X-axis target delta from center of image, in degrees.
    deltaX = tx.getDouble(0.0);
	
  	// get current Y-axis target delta from center of image, with the offset in account, in degrees.
	  deltaY = ty.getDouble(0.0); 
    
    //gets absolute distance from the center of the target.
	  errorX = Math.abs(deltaX);
    
	  
   
    
    double X = deltaY;
    m_velocity = 2234 + (-25.5)*X + (3.3* Math.pow(X, 2.0)) + (-0.231* Math.pow(X, 3.0));
    //Runs shooter at detemined velocity
    m_shooter.shoot(m_velocity, Value.kForward);  
    
   

    //tightened tolerances to +/- 1 degree now that we are 
    //slowing down as the robot is approaching the target
    if(errorX < 1.0){
      if(m_shooter.isWheelAtSpeed()){
        m_tower.driveWholeTower(TowerConstants.STANDARD_TOWER_SPEED);
      
      }
      else{
        
      // else it will stop tower
        m_tower.driveWholeTower(0.0);
      }
    }
    
    System.out.println(errorX + " errorX");
    System.out.println(m_shooter.isWheelAtSpeed() + "at speed");
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //stops tower and shooter 
    m_shooter.stopShooter();
    m_tower.driveWholeTower(0.0);
    LimelightSubsystem.setDriverMode();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
