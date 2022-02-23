// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drive.DriveSubsystem;
import frc.robot.subsystems.Shooter.ShootLowerHub;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.Tower.TowerSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OneShot extends SequentialCommandGroup {
  /** Creates a new OneBallAuto. */
  DriveSubsystem m_drive;
  TowerSubsystem m_tower;
  ShooterSubsystem m_shooter;

  public OneShot(ShooterSubsystem shooter, TowerSubsystem tower) {
    m_shooter = shooter;
    m_tower = tower;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      //shooting command
      new ShootLowerHub(m_shooter, m_tower).withTimeout(5.0)
      //drive forward 3.5 meters
    );
  }
}