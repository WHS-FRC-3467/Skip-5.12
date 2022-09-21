// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Tower.TowerSubsystem;

public class AutoDriveIntake extends CommandBase {
  /** Creates a new AutoDriveIntake. */
  IntakeSubsystem m_intake;
  TowerSubsystem m_tower;
  double m_speed;
  /**
   * 
   * @param intake Intake Subsystem
   * @param tower Tower Subsystem
   * @param speed The speed the intake will be run at -1 to 1
   */
  public AutoDriveIntake(IntakeSubsystem intake, TowerSubsystem tower, double speed) {
    m_intake = intake;
    m_speed = speed;
    m_tower = tower;
    addRequirements(m_intake, m_tower);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //deploys intake
    m_intake.deployIntake();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //drives intake at given speed
    m_intake.driveIntake(-m_speed);
    //runs tower beam brake send to top
    m_tower.sendToTop();
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //stops intake when command ends
    m_intake.driveIntake(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
