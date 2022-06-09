// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autonomous;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Feedback.Cameras.Limelight;
import frc.robot.Subsystems.Drive.DriveSubsystem;
import frc.robot.Subsystems.Drive.PathResetOdometry;
import frc.robot.Subsystems.Drive.TrajectoryFollow;
import frc.robot.Subsystems.Intake.AutoDriveIntake;
import frc.robot.Subsystems.Intake.IntakeSubsystem;
import frc.robot.Subsystems.Shooter.AutoShoot;
import frc.robot.Subsystems.Shooter.LimelightAutoShootTarmac;
import frc.robot.Subsystems.Shooter.Shoot;
import frc.robot.Subsystems.Shooter.ShooterSubsystem;
import frc.robot.Subsystems.Tower.TowerSubsystem;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoBallAutoBattlecryExtraBall extends SequentialCommandGroup {

  IntakeSubsystem m_intake;
  TowerSubsystem m_tower;
  ShooterSubsystem m_shooter;
  DriveSubsystem m_drive;
  Limelight m_limelight;
  public  TwoBallAutoBattlecryExtraBall(IntakeSubsystem intake, TowerSubsystem tower, ShooterSubsystem shooter, DriveSubsystem drive, Limelight limelight){

    m_drive = drive; 
    m_shooter = shooter; 
    m_tower = tower;
    m_intake = intake;
    m_limelight = limelight;
    addCommands(
      new PathResetOdometry("2Ball", m_drive),

      new TrajectoryFollow("2Ball", m_drive).get().raceWith(new AutoDriveIntake(m_intake, m_tower, 1.0)),
      new RunCommand(m_intake::fullRunIntake, m_intake).withTimeout(0.2),

      new LimelightAutoShootTarmac(m_drive, m_shooter, m_tower, m_limelight).withTimeout(4.0),

      new TrajectoryFollow("ExtraBall", m_drive).get().raceWith(new RunCommand(m_intake::fullRunIntake, m_intake)),
      new WaitCommand(0.5),
      new LimelightAutoShootTarmac(m_drive, m_shooter, m_tower, m_limelight).withTimeout(4.0)
    );
  }
}
