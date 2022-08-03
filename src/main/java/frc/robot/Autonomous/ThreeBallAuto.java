// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autonomous;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.Drive.DriveSubsystem;
import frc.robot.Subsystems.Drive.PathResetOdometry;
import frc.robot.Subsystems.Drive.TrajectoryFollow;
import frc.robot.Subsystems.Intake.AutoDriveIntake;
import frc.robot.Subsystems.Intake.IntakeSubsystem;
import frc.robot.Subsystems.Shooter.AutoShoot;
import frc.robot.Subsystems.Shooter.ShooterSubsystem;
import frc.robot.Subsystems.Tower.TowerSubsystem;
import frc.robot.Constants.ShooterConstants;


public class ThreeBallAuto extends SequentialCommandGroup {
  //import subsystem
  IntakeSubsystem m_intake;
  TowerSubsystem m_tower;
  ShooterSubsystem m_shooter;
  DriveSubsystem m_drive;

  /**
   * Constructor for ThreeBallAuto
   * @param drive Drive Subsystem
   * @param shooter Shooter Subsystem
   * @param tower Tower subsystem
   * @param intake Intake Subsystem
   */
  public  ThreeBallAuto(DriveSubsystem drive, ShooterSubsystem shooter, TowerSubsystem tower, IntakeSubsystem intake){
    //Set local variables to member variables
    m_drive = drive; 
    m_shooter = shooter; 
    m_tower = tower;
    m_intake = intake;

    addCommands(
      //Shoot one ball
      new AutoShoot(m_shooter, m_tower, ShooterConstants.kUpperHubFenderVelocity, ShooterConstants.kShooterGains, Value.kReverse),
      //set initial pose
      new PathResetOdometry("3Ball", m_drive),
      //Drive to pick up two balls
      new TrajectoryFollow("3Ball", m_drive).get().raceWith(new AutoDriveIntake(m_intake,m_tower, 1.0)),
      //Shoot two balls
      new AutoShoot(m_shooter, m_tower, ShooterConstants.kUpperHubFenderVelocity, ShooterConstants.kShooterGains, Value.kReverse)
      );
  }
}
