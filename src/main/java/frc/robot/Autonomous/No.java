// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autonomous;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Feedback.Cameras.LimelightSubsystem;
import frc.robot.Subsystems.Drive.DriveSubsystem;
import frc.robot.Subsystems.Drive.PathResetOdometry;
import frc.robot.Subsystems.Drive.TrajectoryFollow;
import frc.robot.Subsystems.Intake.AutoDriveIntake;
import frc.robot.Subsystems.Intake.IntakeSubsystem;
import frc.robot.Subsystems.Shooter.AutoShoot;
import frc.robot.Subsystems.Shooter.LimelightAutoShoot;
import frc.robot.Subsystems.Shooter.ShooterSubsystem;
import frc.robot.Subsystems.Tower.TowerSubsystem;

public class No extends SequentialCommandGroup {
  /** Creates a new FiveBallAuto. */
  //Import subsystems
  ShooterSubsystem m_shooter;
  TowerSubsystem m_tower;
  IntakeSubsystem m_intake;
  DriveSubsystem m_drive;
  LimelightSubsystem m_limelight;

  /**
   * Constructor for FiveBallAuto
   * @param drive Drive Subsystem
   * @param shooter Shooter Subsystem
   * @param tower Tower Subsystem
   * @param intake Intake Subsystem
   *
   */
  public No(DriveSubsystem drive, ShooterSubsystem shooter, TowerSubsystem tower, IntakeSubsystem intake, LimelightSubsystem limelight) {
    //set local variables to member variables
    m_shooter = shooter;
    m_tower = tower;
    m_intake = intake;
    m_drive = drive;
    m_limelight = limelight;

    addCommands(
      //Set initial pose

      new PathResetOdometry("6BallPart1", m_drive),
      new InstantCommand(m_intake::deployIntake, m_intake),
      new TrajectoryFollow("6BallPart1", m_drive).get(),
      new AutoShoot(m_shooter, m_tower, 2025.0, ShooterConstants.kShooterGains, Value.kForward).withTimeout(2.0).raceWith(new RunCommand(m_intake::fullRunIntake, m_intake)),

      new TrajectoryFollow("6BallPart2", m_drive).get().raceWith(new AutoDriveIntake(m_intake, m_tower, 1.0)).withTimeout(1.1),      
      new TrajectoryFollow("5BallPart2", m_drive).get().raceWith(new AutoDriveIntake(m_intake, m_tower, 1.0)).withTimeout(2.0),

      new LimelightAutoShoot(m_shooter, m_tower, m_limelight, m_drive).withTimeout(2.0).raceWith(new RunCommand(m_intake::fullRunIntake, m_intake))

    );
  }
}
