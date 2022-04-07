// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climber;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class A2_LiftAndReach extends SequentialCommandGroup {
  
  ClimberSubsystem m_climberSubsystem;

  public A2_LiftAndReach(ClimberSubsystem climber) {
  
    m_climberSubsystem = climber;

    addCommands(
      new X2_LiftToBar(m_climberSubsystem),
      new X3_ReachToNextBar(m_climberSubsystem)
    );
  }
}
