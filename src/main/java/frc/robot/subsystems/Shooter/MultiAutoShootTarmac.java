// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Tower.TowerSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MultiAutoShootTarmac extends ParallelCommandGroup {
  /** Creates a new MultiAutoShoot. */
  ShooterSubsystem m_shooter;
  TowerSubsystem m_tower;

  public MultiAutoShootTarmac(ShooterSubsystem shooter, TowerSubsystem tower) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    m_shooter = shooter;
    m_tower = tower;
    addCommands(
      //new ShootTarmac(m_shooter, m_tower),
      //new SequentialCommandGroup( new WaitCommand(0.5), new RunCommand(m_tower::driveWholeTower(0.75))
    );
  }
}
