// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Climber;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class A10_DoItAllFast extends SequentialCommandGroup {
  
  ClimberSubsystem m_climberSubsystem;

  public A10_DoItAllFast(ClimberSubsystem climber) {
  
    m_climberSubsystem = climber;

    addCommands(
      new X2_LiftToBar(m_climberSubsystem),
      new ReachToNextBarFast(m_climberSubsystem),
      new HookToNextBarFast(m_climberSubsystem, false),
      new ReachToNextBarFast(m_climberSubsystem),
      new HookToNextBarFast(m_climberSubsystem, true)

    );
  }
}
