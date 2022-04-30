// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autonomous;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoBallAuto extends SequentialCommandGroup {

  IntakeSubsystem m_intake;
  TowerSubsystem m_tower;
  ShooterSubsystem m_shooter;
  DriveSubsystem m_drive;
  public  TwoBallAuto(IntakeSubsystem intake, TowerSubsystem tower, ShooterSubsystem shooter, DriveSubsystem drive){

    m_drive = drive; 
    m_shooter = shooter; 
    m_tower = tower;
    m_intake = intake;

    addCommands(
      new AutoShoot(m_shooter, m_tower, ShooterConstants.kUpperHubFenderVelocity, ShooterConstants.kUpperHubFenderGains, Value.kReverse),
      new InstantCommand(m_intake::intakeDeploy, m_intake),
      new PathResetOdometry("2Ball", m_drive),
      new TrajectoryFollow("2Ball", m_drive).get().raceWith(new AutoDriveIntake(m_intake, m_tower, 1.0)),
      new AutoShoot(m_shooter, m_tower, ShooterConstants.kUpperHubFenderVelocity, ShooterConstants.kUpperHubFenderGains, Value.kReverse)   
      );
  }
}
