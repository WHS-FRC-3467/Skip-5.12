// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Feedback.Cameras.Limelight;
import frc.robot.subsystems.Drive.DriveSubsystem;
import frc.robot.subsystems.Drive.LimelightAim;
import frc.robot.subsystems.Shooter.AutoShootTarmac;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.Tower.TowerSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LimelightOneBall extends SequentialCommandGroup {
  /** Creates a new LimelightOneBall. */
  TowerSubsystem m_tower;
  ShooterSubsystem m_shooter;
  Limelight m_limelight;
  DriveSubsystem m_drive;
  public LimelightOneBall(ShooterSubsystem shooter, TowerSubsystem tower, Limelight limelight, DriveSubsystem drive) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    m_limelight = limelight;
    m_drive = drive;
    m_shooter = shooter;
    m_tower = tower;
    addCommands(
      new LimelightAim(m_drive, m_limelight),
      new AutoShootTarmac(m_shooter, m_tower)
    );
  }
}