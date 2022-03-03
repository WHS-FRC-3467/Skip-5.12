// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClimberConstants;

public class A1_PrepareToClimb extends CommandBase {

  ClimberSubsystem m_climber;

  public A1_PrepareToClimb(ClimberSubsystem climber)
  {
    m_climber = climber;
    addRequirements(m_climber);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {

    // Adjustable Arms to Vertical
    m_climber.extendingClimberVertical();

    // Fixed Arms to Vertical
    m_climber.fixedClimberVertical();

    // Extend Arms to Maximum Length
    m_climber.adjustArmsMagically(ClimberConstants.kFullExtendedPosition);

  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    // If Arms are at Max Length
    return m_climber.areArmsOnTarget();
  }
}
