// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Feedback.Cameras.Limelight;
import frc.robot.subsystems.Drive.DriveSubsystem;
import frc.robot.subsystems.Drive.PathResetOdometry;
import frc.robot.subsystems.Drive.TrajectoryFollow;
import frc.robot.subsystems.Intake.AutoDriveIntake;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Shooter.AutoShoot;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.Tower.TowerSubsystem;

public class ThreeBallAuto extends SequentialCommandGroup {

  IntakeSubsystem m_intake;
  TowerSubsystem m_tower;
  ShooterSubsystem m_shooter;
  DriveSubsystem m_drive;
  Limelight m_limelight;
  public  ThreeBallAuto(IntakeSubsystem intake, TowerSubsystem tower, ShooterSubsystem shooter, DriveSubsystem drive, Limelight limelight){

    m_drive = drive; 
    m_shooter = shooter; 
    m_tower = tower;
    m_intake = intake;
    m_limelight = limelight;

    addCommands(
      new AutoShoot(m_shooter, m_tower).withTimeout(3.0),
      new InstantCommand(m_intake::intakeDeploy, m_intake),
      new PathResetOdometry("3Ball", m_drive),
      new ParallelCommandGroup(      
        new TrajectoryFollow("3Ball", m_drive),
        new AutoDriveIntake(m_intake, m_tower, 1.0)
      ).withTimeout(8.5),

      new AutoShoot(m_shooter, m_tower).withTimeout(3.0)
    );
  }
}
