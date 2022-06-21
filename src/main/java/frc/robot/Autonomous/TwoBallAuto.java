// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autonomous;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Feedback.Cameras.LimelightSubsystem;
import frc.robot.Subsystems.Drive.DriveSubsystem;
import frc.robot.Subsystems.Drive.PathResetOdometry;
import frc.robot.Subsystems.Drive.TrajectoryFollow;
import frc.robot.Subsystems.Intake.AutoDriveIntake;
import frc.robot.Subsystems.Intake.IntakeSubsystem;
import frc.robot.Subsystems.Shooter.LimelightAutoShootTarmac;
import frc.robot.Subsystems.Shooter.ShooterSubsystem;
import frc.robot.Subsystems.Tower.TowerSubsystem;


public class TwoBallAuto extends SequentialCommandGroup {
  //import subsystem
  IntakeSubsystem m_intake;
  TowerSubsystem m_tower;
  ShooterSubsystem m_shooter;
  DriveSubsystem m_drive;
  LimelightSubsystem m_limelight;
  /**
   * Constructor for TwoBallAuto
   * @param drive Drive Subsystem
   * @param shooter Shooter Subsystem
   * @param tower Tower subsystem
   * @param intake Intake Subsystem
   * @param limelight Limelight Subsystem
   */
  public  TwoBallAuto(DriveSubsystem drive, ShooterSubsystem shooter, TowerSubsystem tower, IntakeSubsystem intake, LimelightSubsystem limelight){
    //Set local variables to member variables
    m_drive = drive; 
    m_shooter = shooter; 
    m_tower = tower;
    m_intake = intake;
    m_limelight = limelight;

    addCommands(
      //Set initial pose
      new PathResetOdometry("2Ball", m_drive),
      //Drive back to ball
      new TrajectoryFollow("2Ball", m_drive).get().raceWith(new AutoDriveIntake(m_intake, m_tower, 1.0)),
      //Run intake for 0.2 seconds
      new RunCommand(m_intake::fullRunIntake, m_intake).withTimeout(0.2),
      //Shoot two balls
      new LimelightAutoShootTarmac(m_drive, m_shooter, m_tower, m_limelight)
      );
  }
}
