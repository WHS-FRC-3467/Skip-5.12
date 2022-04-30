// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autonomous;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.Drive.BasicAutoDrive;
import frc.robot.Subsystems.Drive.DriveSubsystem;
import frc.robot.Subsystems.Shooter.AutoShoot;
import frc.robot.Subsystems.Shooter.ShooterSubsystem;
import frc.robot.Subsystems.Tower.TowerSubsystem;
import frc.robot.Constants.ShooterConstants;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SimpleOneBallAuto extends SequentialCommandGroup {
  /** Creates a new OneBallAuto. */
  DriveSubsystem m_drive;
  TowerSubsystem m_tower;
  ShooterSubsystem m_shooter;
  public SimpleOneBallAuto(DriveSubsystem drive, ShooterSubsystem shooter, TowerSubsystem tower) {
    m_drive = drive;
    m_tower = tower;
    m_shooter = shooter;
    addRequirements(m_shooter, m_tower, m_drive);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(m_drive::resetDriveEncoders),
      new AutoShoot(m_shooter, m_tower, ShooterConstants.kUpperHubFenderVelocity, ShooterConstants.kUpperHubFenderGains, Value.kReverse),
      new BasicAutoDrive(drive, 0, 3, 0.0, -1.0, 0.0)
    );
  }
}
