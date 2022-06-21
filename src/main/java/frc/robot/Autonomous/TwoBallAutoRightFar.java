// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autonomous;

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

public class TwoBallAutoRightFar extends SequentialCommandGroup {
  //import subsystem
  ShooterSubsystem m_shooter;
  TowerSubsystem m_tower;
  IntakeSubsystem m_intake;
  DriveSubsystem m_drive;
  LimelightSubsystem m_limelight;
   /**
   * Constructor for TwoBallAutoRightFar
   * @param drive Drive Subsystem
   * @param shooter Shooter Subsystem
   * @param tower Tower subsystem
   * @param intake Intake Subsystem
   * @param limelight Limelight Subsystem
   */

  public TwoBallAutoRightFar(DriveSubsystem drive, ShooterSubsystem shooter, TowerSubsystem tower, IntakeSubsystem intake, LimelightSubsystem limelight) {
    //Set local variables to member variables
    m_shooter = shooter;
    m_tower = tower;
    m_intake = intake;
    m_drive = drive;
    m_limelight = limelight;

    addCommands(
      //Set initial pose
      new PathResetOdometry("5BallPart1", m_drive),
      //Drive to first ball
      new TrajectoryFollow("5BallPart1", m_drive).get().raceWith(new AutoDriveIntake(m_intake, tower, 1.0)),
      //Shoot two balls
      new LimelightAutoShootTarmac(m_drive, m_shooter, m_tower, m_limelight)
    );
  }
}
