// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autonomous;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
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
import frc.robot.Subsystems.Shooter.ShooterSubsystem;
import frc.robot.Subsystems.Tower.TowerSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FiveBallAuto extends SequentialCommandGroup {
  /** Creates a new FiveBallAuto. */
  ShooterSubsystem m_shooter;
  TowerSubsystem m_tower;
  IntakeSubsystem m_intake;
  DriveSubsystem m_drive;
  Limelight m_limelight;

  public FiveBallAuto(ShooterSubsystem shooter, TowerSubsystem tower, IntakeSubsystem intake, DriveSubsystem drive, Limelight limelight) {
    m_shooter = shooter;
    m_tower = tower;
    m_intake = intake;
    m_drive = drive;
    m_limelight = limelight;

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new PathResetOdometry("5BallPart1", m_drive),
      new TrajectoryFollow("5BallPart1", m_drive).get().raceWith(new AutoDriveIntake(m_intake, tower, 1.0)),
      new WaitCommand(0.1),
      new AutoShoot(m_shooter, m_tower, ShooterConstants.kTarmacVelocity, ShooterConstants.kTarmacGains, Value.kForward).withTimeout(2.0).raceWith(new RunCommand(m_intake::fullRunIntake, m_drive)),
      
      new TrajectoryFollow("5BallPart2", m_drive).get().raceWith(new AutoDriveIntake(m_intake, m_tower, 1.0)),
      new WaitCommand(0.1),
      new AutoShoot(m_shooter, m_tower, ShooterConstants.kTarmacVelocity, ShooterConstants.kTarmacGains, Value.kForward).withTimeout(1.5).raceWith(new RunCommand(m_intake::fullRunIntake, m_drive)),

      new TrajectoryFollow("5BallPart3", m_drive).get().raceWith(new AutoDriveIntake(m_intake, m_tower, 1.0)),

      new WaitCommand(0.1),
      new RunCommand(m_intake::fullRunIntake, m_intake).withTimeout(0.5),
   
      new TrajectoryFollow("5BallPart4", m_drive).get(),
      new LimelightAutoShootTarmac(drive, m_shooter, m_tower, limelight)

    );
  }
}
