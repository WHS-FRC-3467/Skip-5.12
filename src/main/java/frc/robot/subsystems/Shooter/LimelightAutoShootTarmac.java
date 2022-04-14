package frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.TowerConstants;
import frc.robot.Feedback.Cameras.Limelight;
import frc.robot.subsystems.Drive.DriveSubsystem;
import frc.robot.subsystems.Drive.LimelightAim;
import frc.robot.subsystems.Tower.DriveTower;
import frc.robot.subsystems.Tower.TowerSubsystem;

public class LimelightAutoShootTarmac extends SequentialCommandGroup {

    DriveSubsystem m_drive;
    ShooterSubsystem m_shooter;
    TowerSubsystem m_tower;
    Limelight m_limelight;

    public LimelightAutoShootTarmac(DriveSubsystem drive, ShooterSubsystem shooter, TowerSubsystem tower, Limelight limelight){
        
    m_drive = drive; 
    m_shooter = shooter; 
    m_tower = tower;
    m_limelight = limelight;

    addCommands(
        new ParallelRaceGroup(
            new ShootTarmac(m_shooter, m_tower),
            new SequentialCommandGroup(
                new LimelightAim(m_drive, m_limelight),
                new DriveTower(m_tower, (() -> TowerConstants.standardTowerSpeed * 0.8)).withTimeout(1.5) //slower to gate balls
            )
        )
    );
    }
    
}
