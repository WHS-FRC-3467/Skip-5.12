// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Shooter;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Util.Gains;

public class Shoot extends CommandBase {
  /** Creates a new ShooterCommand. */
  double m_velocity;
  ShooterSubsystem m_shooter;
  Gains m_gains;
  Value m_hoodPosition;
  public Shoot(ShooterSubsystem shooter, double velocity, Gains gains, Value hoodPosition) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_velocity = velocity;
    m_shooter = shooter;
    m_gains = gains;
    m_hoodPosition = hoodPosition;
    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooter.shoot(m_velocity, m_gains, m_hoodPosition);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.stopShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
