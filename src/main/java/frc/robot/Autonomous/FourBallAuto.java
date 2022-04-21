package frc.robot.Autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Feedback.Cameras.Limelight;
import frc.robot.subsystems.Drive.DriveSubsystem;
import frc.robot.subsystems.Drive.PathResetOdometry;
import frc.robot.subsystems.Drive.TrajectoryFollow2;
import frc.robot.subsystems.Intake.AutoDriveIntake;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Shooter.AutoShootTarmac;
import frc.robot.subsystems.Shooter.LimelightAutoShootTarmac;
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
      new WaitCommand(0.5),
      new InstantCommand(m_intake::intakeDeploy, m_intake),

      new PathResetOdometry("4BallPart1", m_drive),

      new TrajectoryFollow2("4BallPart1", m_drive).get().raceWith(new AutoDriveIntake(m_intake, m_tower, 1.0)),
      new AutoShootTarmac(m_shooter, m_tower).withTimeout(2.5).raceWith(new RunCommand(m_intake::fullRunIntake, m_intake)),

      new TrajectoryFollow2("4BallPart2", m_drive).get().raceWith(new AutoDriveIntake(m_intake, m_tower, 1.0)),
      
      new AutoDriveIntake(m_intake, m_tower, 1.0).withTimeout(0.25),
      new InstantCommand(m_intake::intakeRetract, m_intake),
      new WaitCommand(1.0),
      new InstantCommand(m_intake::intakeDeploy, m_intake),

      new AutoDriveIntake(m_intake, m_tower, 1.0).withTimeout(1.5),

      new TrajectoryFollow2("4BallPart3", m_drive).get(),
      
      new LimelightAutoShootTarmac(m_drive, m_shooter, m_tower, m_limelight)
    );
  }
}
