// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autonomous;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems.Drive.BasicAutoDrive;
import frc.robot.Subsystems.Drive.DriveSubsystem;
import frc.robot.Subsystems.Intake.AutoDriveIntake;
import frc.robot.Subsystems.Intake.IntakeSubsystem;
import frc.robot.Subsystems.Shooter.AutoShoot;
import frc.robot.Subsystems.Shooter.ShooterSubsystem;
import frc.robot.Subsystems.Tower.TowerSubsystem;
import frc.robot.Constants.ShooterConstants;


public class SimpleTwoBallAuto extends SequentialCommandGroup {
  //import subsystem
  DriveSubsystem m_drive;
  ShooterSubsystem m_shooter;
  TowerSubsystem m_tower;
  IntakeSubsystem m_intake;
  /**
   * Constructor for SimpleTwoBallAuto
   * @param drive Drive Subsystem
   * @param shooter Shooter Subsystem
   * @param tower Tower subsystem
   * @param intake Intake Subsystem
   */
  public SimpleTwoBallAuto(DriveSubsystem drive, ShooterSubsystem shooter, TowerSubsystem tower, IntakeSubsystem intake) {
    //Set local variables to member variables
    m_drive = drive;
    m_intake = intake;
    m_tower = tower;
    m_shooter = shooter;

    addCommands(
  //     //Reset drive encoders to zero
  //     new InstantCommand(m_drive::resetDriveEncoders),
  //     //Deploy intake
  //     new InstantCommand(m_intake::deployIntake, m_intake),
  //     //Drive intake for 1 seconds
  //     new AutoDriveIntake(m_intake, m_tower,  1.0).withTimeout(1.0),
  //     //Shoot One ball
  //     new AutoShoot(m_shooter, m_tower, ShooterConstants.kUpperHubFenderVelocity, ShooterConstants.kShooterGains, Value.kReverse),

  //     new ParallelCommandGroup(
  //                               //Drive back to ball and pick up
  //                               new BasicAutoDrive(m_drive, 3.0, -0.878, -1.12, 0.0),
  //                               new AutoDriveIntake(m_intake, m_tower,  1.0).withTimeout(3.5)
  //                             ),
  //     //Resent drive encoders to zero
  //     new InstantCommand(m_drive::resetDriveEncoders),
  //     //Drive back to hub
  //     new BasicAutoDrive(m_drive, 3.0, 0.9, 1.12, 0.0),
  //     //Reset drive encoder
  //     new InstantCommand(m_drive::resetDriveEncoders),
  //     //Wait 0.25 seconds
  //     new WaitCommand(0.25),
  //     //Rotate to goal
  //     new BasicAutoDrive(m_drive, 0.30, 0.0, 0.0, 0.5).withTimeout(0.75),
  //     //Shoot one ball
  //     new AutoShoot(m_shooter, m_tower, ShooterConstants.kUpperHubFenderVelocity, ShooterConstants.kShooterGains, Value.kReverse)    
  );
  }
}
