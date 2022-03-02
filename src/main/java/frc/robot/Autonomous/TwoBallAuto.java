// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drive.BasicAutoDrive;
import frc.robot.subsystems.Drive.DriveSubsystem;
import frc.robot.subsystems.Intake.AutoDriveIntake;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Shooter.ShootUpperHub;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.Tower.TowerSubsystem;

public class TwoBallAuto extends SequentialCommandGroup {
  /** Creates a new TwoBallAuto. */
  DriveSubsystem m_drive;
  ShooterSubsystem m_shooter;
  TowerSubsystem m_tower;
  IntakeSubsystem m_intake;
  public TwoBallAuto(DriveSubsystem drive, ShooterSubsystem shooter, TowerSubsystem tower, IntakeSubsystem intake) {
    m_drive = drive;
    m_intake = intake;
    m_tower = tower;
    m_shooter = shooter;
    addRequirements(m_intake, m_drive, m_shooter, m_tower);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(m_drive::resetDriveEncoders),
      new ShootUpperHub(m_shooter, m_tower).withTimeout(3.0),

      new InstantCommand(m_intake::intakeDeploy, m_intake),
      new ParallelCommandGroup(
                                new BasicAutoDrive(m_drive, 0.0, 3, true, false),
                                new AutoDriveIntake(m_intake, m_tower,  0.75).withTimeout(4)
                              ),
      new InstantCommand(m_drive::resetDriveEncoders),
      new BasicAutoDrive(m_drive, 0.0, 3, false, true),
      new ShootUpperHub(m_shooter, m_tower).withTimeout(3.0)
    );
  }
}
