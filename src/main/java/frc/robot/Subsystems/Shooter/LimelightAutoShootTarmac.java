package frc.robot.Subsystems.Shooter;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TowerConstants;
import frc.robot.Feedback.Cameras.Limelight;
import frc.robot.Subsystems.Drive.DriveSubsystem;
import frc.robot.Subsystems.Drive.LimelightAim;
import frc.robot.Subsystems.Tower.DriveTower;
import frc.robot.Subsystems.Tower.TowerSubsystem;

public class LimelightAutoShootTarmac extends SequentialCommandGroup {

    DriveSubsystem m_drive;
    ShooterSubsystem m_shooter;
    TowerSubsystem m_tower;
    Limelight m_limelight;

    /**
     * 
     * @param drive Drive Subsystem
     * @param shooter Shooter Subsystem
     * @param tower Tower Subsystem
     * @param limelight Limelight Subsystem
     */
    public LimelightAutoShootTarmac(DriveSubsystem drive, ShooterSubsystem shooter, TowerSubsystem tower, Limelight limelight){
        
    m_drive = drive; 
    m_shooter = shooter; 
    m_tower = tower;
    m_limelight = limelight;

    addCommands(
        new ParallelRaceGroup(
            new Shoot(m_shooter, ShooterConstants.kTarmacVelocity, ShooterConstants.kTarmacGains, Value.kForward),
            new SequentialCommandGroup(
                new LimelightAim(m_drive, m_limelight),
                new DriveTower(m_tower, (() -> TowerConstants.standardTowerSpeed * 0.8)).withTimeout(2) //slower to gate balls
            )
        )
    );
    }
    
}