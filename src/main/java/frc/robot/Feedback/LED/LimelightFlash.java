// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Feedback.LED;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class LimelightFlash extends CommandBase {
  /** Creates a new Limelight.
   * Call this command with a timeout. 
   */
  public static NetworkTableInstance table;
  public LimelightFlash() {
    // Use addRequirements() here to declare subsystem dependencies.
    table = NetworkTableInstance.getDefault();

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    table.getTable("limelight").getEntry("ledMode").setNumber(2);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
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
