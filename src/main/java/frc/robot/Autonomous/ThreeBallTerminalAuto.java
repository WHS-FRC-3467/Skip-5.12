// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drive.DriveSubsystem;
import frc.robot.subsystems.Drive.PathResetOdometry;
import frc.robot.subsystems.Drive.TrajectoryFollow;
import frc.robot.subsystems.Intake.AutoDriveIntake;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Shooter.AutoShoot;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.Tower.TowerSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ThreeBallTerminalAuto extends SequentialCommandGroup {
  /** Creates a new ThreeBallTerminal. */
  ShooterSubsystem m_shooter;
  TowerSubsystem m_tower;
  IntakeSubsystem m_intake;
  DriveSubsystem m_drive;
  public ThreeBallTerminalAuto(ShooterSubsystem shooter, TowerSubsystem tower, IntakeSubsystem intake, DriveSubsystem drive) {
    m_intake = intake;
    m_tower = tower;
    m_shooter = shooter;
    m_drive = drive;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AutoShoot(m_shooter, m_tower).withTimeout(3.0),
      new PathResetOdometry("3BallTerminal", m_drive),
      //Timeout Needs to be tuned
      new TrajectoryFollow("3BallTerminal", m_drive).withTimeout(5.0).raceWith(new AutoDriveIntake(m_intake, m_tower, 1.0)),
      new AutoShoot(m_shooter, m_tower).withTimeout(3.0)
    );

  }
}
