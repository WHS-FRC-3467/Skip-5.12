// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drive.AutoDrive;
import frc.robot.subsystems.Drive.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveAuto extends SequentialCommandGroup {
  /** Creates a new DriveAuto. */
  DriveSubsystem m_drive;
  public DriveAuto(DriveSubsystem drive) {
    m_drive = drive;
    addRequirements(m_drive);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      //drives 2 feet at 45 degrees
      new AutoDrive(m_drive, 45, 2)
    );
  }
}
