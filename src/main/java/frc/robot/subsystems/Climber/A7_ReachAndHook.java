// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climber;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class A7_ReachAndHook extends SequentialCommandGroup {
  
  ClimberSubsystem m_climberSubsystem;

  /** Creates a new A10_DoItAll. */
  public A7_ReachAndHook(ClimberSubsystem climber) {
  
    m_climberSubsystem = climber;

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new A3_ReachToNextBar(m_climberSubsystem),
      new A4_HookToNextBar(m_climberSubsystem)
    );
  }
}
