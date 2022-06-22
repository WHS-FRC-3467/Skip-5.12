// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Feedback.Cameras.LimelightSubsystem;
import frc.robot.Subsystems.Drive.DriveSubsystem;
import frc.robot.Subsystems.Shooter.LimelightAutoShootTarmac;
import frc.robot.Subsystems.Shooter.ShooterSubsystem;
import frc.robot.Subsystems.Tower.TowerSubsystem;

public class LimelightOneBall extends SequentialCommandGroup {
  //Import Subsystems
  TowerSubsystem m_tower;
  ShooterSubsystem m_shooter;
  LimelightSubsystem m_limelight;
  DriveSubsystem m_drive;
  /**
   * Constructor for LimelightOneBall
   * @param drive Drive Subsystem
   * @param shooter Shooter Subsystem
   * @param tower Tower Subsystem
   * @param limelight Limelight Subsystem
   */
  public LimelightOneBall(DriveSubsystem drive, ShooterSubsystem shooter, TowerSubsystem tower, LimelightSubsystem limelight) {
    //Set Local variables to member variables
    m_limelight = limelight;
    m_drive = drive;
    m_shooter = shooter;
    m_tower = tower;
    addCommands(
      //Use limelight to drive back into position
      //Shoot one ball
      new LimelightAutoShootTarmac(m_drive, m_shooter, m_tower, m_limelight)
    );
  }
}
