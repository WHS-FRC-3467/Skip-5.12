// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Shooter;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Shoot extends CommandBase {
  /** Creates a new ShooterCommand. */
  //Intialize member variables
  double m_velocity;
  ShooterSubsystem m_shooter;
  Value m_hoodPosition;
  /**
   * @param shooter Shooter Subsystem 
   * @param velocity Velocity of shooter in RPM
   * @param gains Gains for shooter
   * @param hoodPosition The hood position in Value.kFoward (deployed) or Value.kReverse (retracted)
   */
  public Shoot(ShooterSubsystem shooter, double velocity, Value hoodPosition) {
    //Sets local variables to member variables
    m_velocity = velocity;
    m_shooter = shooter;
    m_hoodPosition = hoodPosition;
    //addes requirements for shooter
    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //runs shooter
    m_shooter.shoot(m_velocity, m_hoodPosition);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //stops shooter if command ends
    m_shooter.stopShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
