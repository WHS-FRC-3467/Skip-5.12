// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveIntake extends CommandBase {
  /** Creates a new DriveIntake. */
  
  DoubleSupplier m_forwardspeed, m_backspeed;
  IntakeSubsystem m_intake;
  
  public DriveIntake(IntakeSubsystem intake, DoubleSupplier forwardspeed, DoubleSupplier backspeed) {
    m_intake = intake;
    m_forwardspeed = forwardspeed;
    m_backspeed = backspeed;
    addRequirements(m_intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_forwardspeed.getAsDouble() > 0.2){
       m_intake.driveIntake(m_forwardspeed.getAsDouble());
    }
    if(m_backspeed.getAsDouble() > 0.2){
      m_intake.driveIntake(-m_backspeed.getAsDouble());
    }
   if(m_backspeed.getAsDouble() < 0.2 && m_forwardspeed.getAsDouble() < 0.2){
      m_intake.driveIntake(0.0);
   }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.driveIntake(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
