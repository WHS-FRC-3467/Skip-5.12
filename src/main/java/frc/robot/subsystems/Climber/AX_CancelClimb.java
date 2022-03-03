// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climber;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class AX_CancelClimb extends InstantCommand {

  public AX_CancelClimb(ClimberSubsystem climber) {
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Climb has been CANCELLED");
  }
}
