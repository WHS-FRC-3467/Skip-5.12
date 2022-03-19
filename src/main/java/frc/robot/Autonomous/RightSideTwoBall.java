// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Drive.BasicAutoDrive;
import frc.robot.subsystems.Drive.DriveSubsystem;
import frc.robot.subsystems.Drive.PathResetOdometry;
import frc.robot.subsystems.Drive.TrajectoryFollow;
import frc.robot.subsystems.Intake.AutoDriveIntake;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Shooter.AutoShoot;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.Tower.TowerSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RightSideTwoBall extends SequentialCommandGroup {
  /** Creates a new RightSideTwoBall. */
  IntakeSubsystem m_intake;
  TowerSubsystem m_tower;
  ShooterSubsystem m_shooter;
  DriveSubsystem m_drive;

  public RightSideTwoBall(IntakeSubsystem intake, TowerSubsystem tower, ShooterSubsystem shooter, DriveSubsystem drive) {
    m_intake = intake;
    m_tower = tower;
    m_shooter = shooter;
    m_drive = drive;
    addRequirements(m_intake, m_drive, m_shooter, m_tower);

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AutoShoot(m_shooter, m_tower).withTimeout(3.0),
      new PathResetOdometry("RightSide2Ball", m_drive),

       new ParallelCommandGroup( 

              new TrajectoryFollow("RightSide2Ball", m_drive),
              new AutoDriveIntake(m_intake, m_tower,  1.0)).withTimeout(5.0),

      new InstantCommand(m_drive::resetDriveEncoders),
      new WaitCommand(0.25),
      new BasicAutoDrive(m_drive, 0.0, 0.1, 0.0, 0.0, 0.5).withTimeout(0.75),
                        
      new AutoShoot(m_shooter, m_tower).withTimeout(3.0)

    );
  }
}
