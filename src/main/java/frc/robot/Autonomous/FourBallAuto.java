package frc.robot.Autonomous;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Feedback.Cameras.Limelight;
import frc.robot.Subsystems.Drive.DriveSubsystem;
import frc.robot.Subsystems.Drive.LimelightAim;
import frc.robot.Subsystems.Drive.PathResetOdometry;
import frc.robot.Subsystems.Drive.TrajectoryFollow;
import frc.robot.Subsystems.Intake.AutoDriveIntake;
import frc.robot.Subsystems.Intake.IntakeSubsystem;
import frc.robot.Subsystems.Shooter.AutoShoot;
import frc.robot.Subsystems.Shooter.LimelightAutoShootTarmac;
import frc.robot.Subsystems.Shooter.ShooterSubsystem;
import frc.robot.Subsystems.Tower.TowerSubsystem;


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

      new TrajectoryFollow("4BallPart1", m_drive).get().raceWith(new AutoDriveIntake(m_intake, m_tower, 1.0)),
      new AutoShoot(m_shooter, m_tower, ShooterConstants.kTarmacVelocity, ShooterConstants.kTarmacGains, Value.kForward).withTimeout(2.5).raceWith(new RunCommand(m_intake::fullRunIntake, m_intake)),

      new TrajectoryFollow("4BallPart2", m_drive).get().raceWith(new AutoDriveIntake(m_intake, m_tower, 1.0)),
      
      // // LEAVE COMMENTED new AutoDriveIntake(m_intake, m_tower, 1.0).withTimeout(0.25),
      new InstantCommand(m_intake::intakeRetract, m_intake),
      new WaitCommand(0.5),

      new AutoDriveIntake(m_intake, m_tower, 1.0).withTimeout(1.5),

      new TrajectoryFollow("4BallPart3", m_drive).get(),
      
      new LimelightAim(m_drive, m_limelight, false, true),
      new LimelightAutoShootTarmac(drive, m_shooter, m_tower, limelight)
    );
  }
}
