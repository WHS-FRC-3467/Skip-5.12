// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Shooter;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.TowerConstants;
import frc.robot.Feedback.Cameras.LimelightSubsystem;
import frc.robot.Subsystems.Drive.DriveSubsystem;
import frc.robot.Subsystems.Tower.TowerSubsystem;

public class LimelightAutoShoot extends CommandBase {
  /** Creates a new LimelightShoot. */

  //Must be followed by Limelight Aim

  TowerSubsystem m_tower;
  ShooterSubsystem m_shooter;
  DriveSubsystem m_drive;
  LimelightSubsystem m_limelight;
  double m_velocity, deltaY, count, m_rotation, m_translationY, m_targetThreshold, deltaX, deltaYTargetTarmac, errorX, errorY; 

  NetworkTable table;
  NetworkTableEntry tx, ty, ta, tv;
  boolean m_waitForTarget, m_hasTarget;


  /**
   * @param shooter Shooter Subsystem
   * @param tower Tower Subsystem
   * @param limelight LimelightSubsystem
   */
  public LimelightAutoShoot(ShooterSubsystem shooter, TowerSubsystem tower, LimelightSubsystem limelight, DriveSubsystem drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = shooter;
    m_limelight = limelight; 
    m_tower = tower; 
    m_drive = drive;
    addRequirements(m_shooter, m_tower, m_limelight, m_drive);
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

    //Updates network table variables
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    tv = table.getEntry("tv");
    m_hasTarget = tv.getDouble(1.0) == 1.0;
	  //removed ta since it's unused and slows the loop
	
    // get current X-axis target delta from center of image, in degrees.
    deltaX = tx.getDouble(0.0);
	
  	// get current Y-axis target delta from center of image, with the offset in account, in degrees.
	  deltaY = ty.getDouble(0.0); 
    
    //gets absolute distance from the center of the target.
	  errorX = Math.abs(deltaX);
	  
    //fixed clamp logic here
    //increased limits to 1.5 now that we are slowing down approaching target
    //feel free to lower these, or divide by a bigger number if oscillations occur
    m_rotation = Math.max(-1.5, Math.min(1.5, deltaX/6.0));

    m_drive.drive(new ChassisSpeeds(0, 0, -m_rotation));
    
    //Zero is tarmac velocity
    m_velocity = 2270.8 * Math.pow(Math.E, -0.01 * deltaY);

    //Runs shooter at detemined velocity
    m_shooter.shoot(m_velocity, Value.kForward);  

    //tightened tolerances to +/- 1 degree now that we are 
    //slowing down as the robot is approaching the target
    if((errorX < 0.5) && m_hasTarget){
      if(m_shooter.isWheelAtSpeed()){
        m_tower.driveWholeTower(TowerConstants.STANDARD_TOWER_SPEED);
      }
      else{
      // else it will stop tower
        m_tower.driveWholeTower(0.0);
      }
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
    return false;
  }
}
