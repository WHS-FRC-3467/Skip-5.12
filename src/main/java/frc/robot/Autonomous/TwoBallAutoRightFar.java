// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Feedback.Cameras.Limelight;
import frc.robot.subsystems.Drive.DriveSubsystem;
import frc.robot.subsystems.Drive.PathResetOdometry;
import frc.robot.subsystems.Drive.TrajectoryFollow2;
import frc.robot.subsystems.Intake.AutoDriveIntake;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Shooter.LimelightAutoShootTarmac;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.Tower.TowerSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoBallAutoRightFar extends SequentialCommandGroup {
  /** Creates a new TwoBallAutoRightFar. */
  DriveSubsystem m_drive;
  TowerSubsystem m_tower; 
  IntakeSubsystem m_intake;
  Limelight m_limelight;
  ShooterSubsystem m_shooter;
  public TwoBallAutoRightFar(DriveSubsystem drive, IntakeSubsystem intake, TowerSubsystem tower, ShooterSubsystem shooter, Limelight limelight) {
    m_drive = drive;
    m_tower = tower;
    m_intake = intake;
    m_limelight = limelight;
    m_shooter = shooter;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new PathResetOdometry("RightSide2BallFar", m_drive),
      new InstantCommand(m_intake::intakeDeploy, m_intake),

      new TrajectoryFollow2("RightSide2BallFar", m_drive).get().raceWith(new AutoDriveIntake(m_intake, m_tower, 1.0)),
      new InstantCommand(m_intake::intakeRetract, m_intake),

      new LimelightAutoShootTarmac(m_drive, m_shooter, m_tower, m_limelight)

    );
  }
}
