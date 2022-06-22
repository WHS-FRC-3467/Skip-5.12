// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
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

public class TwoBallAutoRightClose extends SequentialCommandGroup {
  //import subsystem
  DriveSubsystem m_drive;
  TowerSubsystem m_tower; 
  IntakeSubsystem m_intake;
  LimelightSubsystem m_limelight;
  ShooterSubsystem m_shooter;
  /**
   * Constructor for TwoBallAutoRightClose
   * @param drive Drive Subsystem
   * @param shooter Shooter Subsystem
   * @param tower Tower subsystem
   * @param intake Intake Subsystem
   * @param limelight Limelight Subsystem
   */

  public TwoBallAutoRightClose(DriveSubsystem drive, ShooterSubsystem shooter, TowerSubsystem tower, IntakeSubsystem intake, LimelightSubsystem limelight) {
    //Set local variables to member variables
    m_drive = drive;
    m_tower = tower;
    m_intake = intake;
    m_limelight = limelight;
    m_shooter = shooter;

    addCommands(
      //set initial pose
      new PathResetOdometry("4BallPart1", m_drive),
      //drive to first ball
      new TrajectoryFollow("4BallPart1", m_drive).get().raceWith(new AutoDriveIntake(m_intake, m_tower, 1.0)),
      //Retract intake
      new InstantCommand(m_intake::retractIntake, m_intake),
      //Shoot two balls
      new LimelightAutoShootTarmac(m_drive, m_shooter, m_tower, m_limelight)

    );
  }
}
