package frc.robot.subsystems.Drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Control.XboxControllerEE;

import java.util.function.DoubleSupplier;

public class SwerveDrive extends CommandBase{
      //Initialize Variables
    DriveSubsystem m_driveSubsystem;
    DoubleSupplier m_translationXSupplier;
    DoubleSupplier m_translationYSupplier;
    DoubleSupplier m_rotationSupplier;
    private final XboxControllerEE m_driverController = new XboxControllerEE(0);

    //Constructor for SwerveDrive
    public SwerveDrive(DriveSubsystem driveSubsystem, DoubleSupplier translationXSupplier, DoubleSupplier translationYSupplier, DoubleSupplier rotationSupplier) {
        m_driveSubsystem = driveSubsystem;
        m_translationXSupplier = translationXSupplier;
        m_translationYSupplier = translationYSupplier;
        m_rotationSupplier = rotationSupplier;
 
        addRequirements(m_driveSubsystem);
    }

    @Override
    public void execute() {
        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
        if(m_driverController.getBButton()){
            m_driveSubsystem.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    m_translationXSupplier.getAsDouble() * DriveConstants.precisionSpeed,
                    m_translationYSupplier.getAsDouble() * DriveConstants.precisionSpeed,
                    m_rotationSupplier.getAsDouble() *DriveConstants.precisionSpeed,
                    m_driveSubsystem.getGyroscopeRotation()

                )
            );  
        }
        else{
            m_driveSubsystem.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    m_translationXSupplier.getAsDouble(),
                    m_translationYSupplier.getAsDouble(),
                    m_rotationSupplier.getAsDouble(),
                    m_driveSubsystem.getGyroscopeRotation()
                )
            ); 
        }
        
    }

    @Override
    public void end(boolean interrupted) {
        m_driveSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}
