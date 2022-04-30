// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Feedback.Cameras.Limelight;
import frc.robot.Subsystems.Drive.DriveSubsystem;
import frc.robot.Subsystems.Drive.LimelightAim;
import frc.robot.Subsystems.Drive.PathResetOdometry;
import frc.robot.Subsystems.Drive.TrajectoryFollow;
import frc.robot.Subsystems.Intake.IntakeSubsystem;
import frc.robot.Subsystems.Tower.TowerSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LimelightTest extends SequentialCommandGroup {
  /** Creates a new LimelightTest. */
  Limelight m_limelight;
  DriveSubsystem m_drive;
  IntakeSubsystem m_intake;
  TowerSubsystem m_tower;
  public LimelightTest(Limelight limelight, DriveSubsystem drive, IntakeSubsystem intake, TowerSubsystem tower) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    m_intake = intake;
    m_drive = drive;
    m_limelight = limelight;
    m_tower = tower;
    addCommands(
      new PathResetOdometry("4BallPart1", m_drive),
      new TrajectoryFollow("4BallPart1", m_drive).get(),
      new LimelightAim(m_drive, m_limelight),
      new TrajectoryFollow("4BallPart2", m_drive).get()
    );
  }
}
