// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Drive.BasicAutoDrive;
import frc.robot.subsystems.Drive.DriveSubsystem;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Shooter.AutoShoot;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.Tower.TowerSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoBallAuto extends SequentialCommandGroup {
  /** Creates a new TwoBallAuto. */
  DriveSubsystem m_drive;
  IntakeSubsystem m_intake;
  TowerSubsystem m_tower;
  ShooterSubsystem m_shooter;
  public TwoBallAuto(DriveSubsystem drive, IntakeSubsystem intake, ShooterSubsystem shooter, TowerSubsystem tower) {
    m_drive = drive;
    m_intake = intake;
    m_shooter = shooter;
    m_tower = tower;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AutoShoot(m_shooter, m_tower, ShooterConstants.lowerHubVelocity).withTimeout(5.0),
      new InstantCommand(m_intake::intakeDeploy),
      new BasicAutoDrive(drive, 14.5, 3.3, true),
      new BasicAutoDrive(drive, 14.5, -3.3, false),
      new AutoShoot(m_shooter, m_tower, ShooterConstants.lowerHubVelocity).withTimeout(5.0)
    );
  }
}
