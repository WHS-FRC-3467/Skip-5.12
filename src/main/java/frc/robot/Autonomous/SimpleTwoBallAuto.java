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
  /** Creates a new TwoBallAuto. */
  DriveSubsystem m_drive;
  ShooterSubsystem m_shooter;
  TowerSubsystem m_tower;
  IntakeSubsystem m_intake;
  public SimpleTwoBallAuto(DriveSubsystem drive, ShooterSubsystem shooter, TowerSubsystem tower, IntakeSubsystem intake) {
    m_drive = drive;
    m_intake = intake;
    m_tower = tower;
    m_shooter = shooter;
    addRequirements(m_intake, m_drive, m_shooter, m_tower);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(

      new InstantCommand(m_drive::resetDriveEncoders),
      new InstantCommand(m_intake::intakeDeploy, m_intake),
      new AutoDriveIntake(m_intake, m_tower,  1.0).withTimeout(1.0),

      new AutoShoot(m_shooter, m_tower, ShooterConstants.kUpperHubFenderVelocity, ShooterConstants.kUpperHubFenderGains, Value.kReverse),

      new ParallelCommandGroup(
                                new BasicAutoDrive(m_drive, 0.0, 3.0, -0.878, -1.12, 0.0),
                                new AutoDriveIntake(m_intake, m_tower,  1.0).withTimeout(3.5)
                              ),
      new InstantCommand(m_drive::resetDriveEncoders),
      new BasicAutoDrive(m_drive, 0.0, 3.0, 0.9, 1.12, 0.0),
      new InstantCommand(m_drive::resetDriveEncoders),
      new WaitCommand(0.25),
      new BasicAutoDrive(m_drive, 0.0, 0.30, 0.0, 0.0, 0.5).withTimeout(0.75),
      new AutoShoot(m_shooter, m_tower, ShooterConstants.kUpperHubFenderVelocity, ShooterConstants.kUpperHubFenderGains, Value.kReverse)    );
  }
}
