package frc.robot.Autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Feedback.Cameras.Limelight;
import frc.robot.subsystems.Drive.DriveSubsystem;
import frc.robot.subsystems.Drive.PathResetOdometry;
import frc.robot.subsystems.Drive.TrajectoryFollow;
import frc.robot.subsystems.Intake.AutoDriveIntake;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Shooter.AutoShootTarmac;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.Tower.TowerSubsystem;


public class FourBallAuto extends SequentialCommandGroup {

  ShooterSubsystem m_shooter;
  TowerSubsystem m_tower;
  IntakeSubsystem m_intake;
  DriveSubsystem m_drive;
  Limelight m_limelight;

  public FourBallAuto(ShooterSubsystem shooter, TowerSubsystem tower, IntakeSubsystem intake, DriveSubsystem drive, Limelight limelight) {
    m_shooter = shooter;
    m_tower = tower;
    m_intake = intake;
    m_drive = drive;
    m_limelight = limelight;

    addCommands(
      new InstantCommand(m_intake::intakeDeploy, m_intake),

      new PathResetOdometry("3Ball", m_drive),
      new ParallelCommandGroup(      
        new TrajectoryFollow("3Ball", m_drive),
        new AutoDriveIntake(m_intake, m_tower, 1.0)
      ).withTimeout(8.5),
       
      new AutoShootTarmac(m_shooter, m_tower).withTimeout(3.0),

      new ParallelCommandGroup(      
        new TrajectoryFollow("4BallPart2", m_drive),
        new AutoDriveIntake(m_intake, m_tower, 1.0)
      ).withTimeout(3.2),
      new WaitCommand(2.0),

      new TrajectoryFollow("4BallPart3", m_drive).withTimeout(3.1),
      
      new AutoShootTarmac(m_shooter, m_tower).withTimeout(3.0)
    );
  }
}
