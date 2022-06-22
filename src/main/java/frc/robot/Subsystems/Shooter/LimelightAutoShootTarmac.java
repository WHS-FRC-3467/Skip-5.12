package frc.robot.Subsystems.Shooter;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Feedback.Cameras.LimelightSubsystem;
import frc.robot.Subsystems.Drive.DriveSubsystem;
import frc.robot.Subsystems.Drive.LimelightAim;
import frc.robot.Subsystems.Tower.TowerSubsystem;

public class LimelightAutoShootTarmac extends SequentialCommandGroup {
    //Initializes variable
    DriveSubsystem m_drive;
    ShooterSubsystem m_shooter;
    TowerSubsystem m_tower;
    LimelightSubsystem m_limelight;

    /**
     * Constructor for Limelight Auto Shoot Tarmac
     * @param drive Drive Subsystem
     * @param shooter Shooter Subsystem
     * @param tower Tower Subsystem
     * @param limelight Limelight Subsystem
     */
    public LimelightAutoShootTarmac(DriveSubsystem drive, ShooterSubsystem shooter, TowerSubsystem tower, LimelightSubsystem limelight){
    //sets local variables to member variables
    m_drive = drive; 
    m_shooter = shooter; 
    m_tower = tower;
    m_limelight = limelight;

    addCommands(
        //Starts shooter and runs limelight aim
        new ParallelRaceGroup(
            new Shoot(m_shooter, ShooterConstants.kTarmacVelocity, ShooterConstants.kTarmacGains, Value.kForward),
            new LimelightAim(m_drive, m_limelight)
        ),
        //Runs auto shoot
        new AutoShoot(m_shooter, m_tower, ShooterConstants.kTarmacVelocity, ShooterConstants.kTarmacGains, Value.kForward)


        
    );

    }
    
}
