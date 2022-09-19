// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Intake;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveIntake extends CommandBase {
  /** Creates a new DriveIntake. */
  
  DoubleSupplier m_forwardspeed, m_backspeed;
  IntakeSubsystem m_intake;
  /**
   * 
   * @param intake Intake Subsystem
   * @param forwardspeed The speed the intake will be driven at  forward 0 to 1 
   * @param backspeed The speed the intake will be driven at in reverse 0 to 1
   */
  public DriveIntake(IntakeSubsystem intake, DoubleSupplier forwardspeed, DoubleSupplier backspeed) {
    m_intake = intake;
    m_forwardspeed = forwardspeed;
    m_backspeed = backspeed;
    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //if forward speed is greater than 0.2 runs intake at forwardSpeed and deploys intake
    if(m_forwardspeed.getAsDouble() > 0.2){
       m_intake.driveIntake(-m_forwardspeed.getAsDouble());
      //  m_intake.deployIntake();
    }
    //if forward speed is greater than 0.2 runs intake at backSpeed and deploys intake 
    if(m_backspeed.getAsDouble() > 0.2){
      m_intake.driveIntake(m_backspeed.getAsDouble());
      // m_intake.deployIntake();
    }
    //If both speeds are less that 0.2 then it keeps intake up and doesn't drive intake
    if(m_backspeed.getAsDouble() < 0.2 && m_forwardspeed.getAsDouble() < 0.2){
      m_intake.driveIntake(0.0);
      // m_intake.retractIntake();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //stops intake if command ends
    m_intake.driveIntake(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
