package frc.robot.subsystems.Drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Control.XBoxControllerEE;

import java.util.function.DoubleSupplier;

public class SwerveDrive extends CommandBase{
      //Initialize Variables
    DriveSubsystem m_driveSubsystem;
    DoubleSupplier m_translationXSupplier;
    DoubleSupplier m_translationYSupplier;
    DoubleSupplier m_rotationSupplier;
    Boolean m_precisionMode;
    private final XBoxControllerEE m_driverController = new XBoxControllerEE(0);
    
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
        if(m_driverController.getLeftBumper()){
            m_driveSubsystem.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    m_driveSubsystem.modifyAxis(m_translationYSupplier.getAsDouble() * DriveConstants.precisionSpeed, 2),
                    m_driveSubsystem.modifyAxis(m_translationXSupplier.getAsDouble() * DriveConstants.precisionSpeed, 2),
                    m_driveSubsystem.modifyAxis(m_rotationSupplier.getAsDouble() * 0.1, 2),
                    m_driveSubsystem.getGyroscopeRotation()

                )
            );  
        }
        else if(m_driverController.getRightBumper()){
            m_driveSubsystem.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    m_driveSubsystem.modifyAxis(m_translationYSupplier.getAsDouble() * 0.5, 2),
                    m_driveSubsystem.modifyAxis(m_translationXSupplier.getAsDouble() * 0.5, 2),
                    m_driveSubsystem.modifyAxis(m_rotationSupplier.getAsDouble() * 0.1, 2),
                    m_driveSubsystem.getGyroscopeRotation()

                )
            );  
        }
        else{
            m_driveSubsystem.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    m_driveSubsystem.modifyAxis(m_translationYSupplier.getAsDouble(), 2),
                    m_driveSubsystem.modifyAxis(m_translationXSupplier.getAsDouble(), 2),
                    m_driveSubsystem.modifyAxis(m_rotationSupplier.getAsDouble()* 0.5, 2),
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
