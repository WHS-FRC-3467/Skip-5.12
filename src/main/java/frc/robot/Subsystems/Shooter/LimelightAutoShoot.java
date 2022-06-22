// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Feedback.Cameras.LimelightSubsystem;
import frc.robot.Subsystems.Drive.DriveSubsystem;
import frc.robot.Subsystems.Drive.LimelightAim;
import frc.robot.Subsystems.Tower.TowerSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LimelightAutoShoot extends SequentialCommandGroup {
  /** Creates a new LimelightAutoShoot. */
  //Initializes variables
  LimelightSubsystem m_limelight;
  DriveSubsystem m_drive;
  ShooterSubsystem m_shooter;
  TowerSubsystem m_tower;
  /**
   * Constructor for LimelightAutoShoot
   * @param drive Drive subsystem
   * @param limelight Limelight
   * @param shooter Shooter Subsystem
   * @param tower Tower Subsystem
   */
  public LimelightAutoShoot(DriveSubsystem drive, LimelightSubsystem limelight, ShooterSubsystem shooter, TowerSubsystem tower) {
    
    //Sets local variables to member variables
    m_drive = drive;
    m_limelight = limelight;
    m_shooter = shooter;
    m_tower = tower; 
    addCommands(
      //lines up limelight to goal
      new LimelightAim(m_drive, m_limelight, true, false),
      //Runs limelight shoot
      new LimelightShoot(m_shooter, tower, m_limelight)
    );
  }
}
