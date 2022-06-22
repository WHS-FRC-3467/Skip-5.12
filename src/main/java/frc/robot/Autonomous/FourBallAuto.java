package frc.robot.Autonomous;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Feedback.Cameras.LimelightSubsystem;
import frc.robot.Subsystems.Drive.DriveSubsystem;
import frc.robot.Subsystems.Drive.PathResetOdometry;
import frc.robot.Subsystems.Drive.TrajectoryFollow;
import frc.robot.Subsystems.Intake.AutoDriveIntake;
import frc.robot.Subsystems.Intake.IntakeSubsystem;
import frc.robot.Subsystems.Shooter.AutoShoot;
import frc.robot.Subsystems.Shooter.LimelightAutoShootTarmac;
import frc.robot.Subsystems.Shooter.ShooterSubsystem;
import frc.robot.Subsystems.Tower.TowerSubsystem;


public class FourBallAuto extends SequentialCommandGroup {
  //import Subsystems 
  ShooterSubsystem m_shooter;
  TowerSubsystem m_tower;
  IntakeSubsystem m_intake;
  DriveSubsystem m_drive;
  LimelightSubsystem m_limelight;
  /**
   * Constructor for FourBallAuto
   * @param drive Drive Subsystem
   * @param shooter Shooter Subsystem
   * @param tower Tower subsystem
   * @param intake Intake Subsystem
   * @param limelight Limelight Subsystem
   */
  public FourBallAuto(DriveSubsystem drive, ShooterSubsystem shooter, TowerSubsystem tower, IntakeSubsystem intake, LimelightSubsystem limelight){
    //set local variables to member variables
    m_shooter = shooter;
    m_tower = tower;
    m_intake = intake;
    m_drive = drive;
    m_limelight = limelight;

    addCommands(
      //Wait 0.5 seconds
      new WaitCommand(0.5),
      //Deploy intake
      new InstantCommand(m_intake::deployIntake, m_intake),
      //Set initial pose
      new PathResetOdometry("4BallPart1", m_drive),
      //Drive to first Ball
      new TrajectoryFollow("4BallPart1", m_drive).get().raceWith(new AutoDriveIntake(m_intake, m_tower, 1.0)),
      //Shoot two balls
      new AutoShoot(m_shooter, m_tower, ShooterConstants.kTarmacVelocity, ShooterConstants.kTarmacGains, Value.kForward).withTimeout(2.5).raceWith(new RunCommand(m_intake::fullRunIntake, m_intake)),
      //Drive to terminal
      new TrajectoryFollow("4BallPart2", m_drive).get().raceWith(new AutoDriveIntake(m_intake, m_tower, 1.0)),
      //Retract intake
      new InstantCommand(m_intake::retractIntake, m_intake),
      //Wait 0.5 seconds
      new WaitCommand(0.5),
      //Drive intake for 1.5 seconds
      new AutoDriveIntake(m_intake, m_tower, 1.0).withTimeout(1.5),
      //Drive to hub
      new TrajectoryFollow("4BallPart3", m_drive).get(),
      //Shoot two balls
      new LimelightAutoShootTarmac(drive, m_shooter, m_tower, limelight)
    );
  }
}
