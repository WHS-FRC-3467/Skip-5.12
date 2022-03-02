// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClimberConstants;
// import frc.robot.subsystems.Intake.IntakeSubsystem;

public class A1_PrepareToClimb extends CommandBase {

  ClimberSubsystem m_climber;
//  IntakeSubsystem m_intake;

  public A1_PrepareToClimb(ClimberSubsystem climber) //, IntakeSubsystem intake)
  {
    m_climber = climber;
//    m_intake = intake;
    addRequirements(m_climber);
  }

  @Override
  public void initialize() {

    // Deploy Intake
    // m_intake.intakeDeploy();

  }

  @Override
  public void execute() {

    // Angle Adjustable Arms
    m_climber.extendingClimberAngled();

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
