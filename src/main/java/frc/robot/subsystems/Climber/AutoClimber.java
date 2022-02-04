// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.Climber;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ClimberConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoClimber extends SequentialCommandGroup {
  /** Creates a new AutoClimber. */
  ClimberSubsystem m_climber;
  public AutoClimber(ClimberSubsystem climber) {
      m_climber = climber;
    addCommands(
      new PIDClimberPosition(m_climber, ClimberConstants.extendedPosition),
      new InstantCommand(m_climber::fixedClimberForward),
      new PIDClimberPosition(m_climber, ClimberConstants.retractedPostion),
      new InstantCommand(m_climber::fixedClimberReverse),
      new ParallelCommandGroup(new PIDClimberPosition(m_climber, ClimberConstants.extendedPosition), new InstantCommand(m_climber::fixedClimberForward)),
      new PIDClimberPosition(m_climber, ClimberConstants.retractedPostion),
      new InstantCommand(m_climber::fixedClimberReverse)
    );
  }
}
