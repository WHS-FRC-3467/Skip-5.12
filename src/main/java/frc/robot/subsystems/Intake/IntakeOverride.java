// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeOverride extends CommandBase {
  IntakeSubsystem m_intake;
  boolean m_deployed;
  public IntakeOverride(IntakeSubsystem intake, boolean deployed) {
    m_intake = intake;
    m_deployed = deployed;
    addRequirements(m_intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }
  @Override
  public void execute() {
    if(m_deployed == true){
      m_intake.intakeDeploy();
    }
    else if(m_deployed == false){
      m_intake.intakeRetract();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
