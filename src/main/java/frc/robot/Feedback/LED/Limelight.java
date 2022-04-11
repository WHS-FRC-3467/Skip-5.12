// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Feedback.LED;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class Limelight extends CommandBase {
  /** Creates a new Limelight. */
  private TowerSubsystem m_tower;
  public Limelight(TowerSubsystem m_tower) {
    // Use addRequirements() here to declare subsystem dependencies.
    public static NetworkTableInstance table = NetworkTableInstance.getDefault();
    this.m_tower = m_tower;
    addRequirements(m_tower);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!m_tower.m_entryBeamBreak) {
      table.getTable("limelight").getEntry("ledMode").setNumber(2);
      //There is probably a better way to do this
      thread.sleep(2000);
    }
    else {
      table.getTable("limelight").getEntry("ledMode").setNumber(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    table.getTable("limelight").getEntry("ledMode").setNumber(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
