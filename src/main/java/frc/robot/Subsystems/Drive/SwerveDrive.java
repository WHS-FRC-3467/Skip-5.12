package frc.robot.Subsystems.Drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Control.XBoxControllerEE;
import frc.robot.Util.ModifyAxis;

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
    /**
     * 
     * @param driveSubsystem drive Subsystem
     * @param translationXSupplier X Value
     * @param translationYSupplier Y value
     * @param rotationSupplier Rotation value
     */
    public SwerveDrive(DriveSubsystem driveSubsystem, DoubleSupplier translationXSupplier, DoubleSupplier translationYSupplier, DoubleSupplier rotationSupplier) {
        m_driveSubsystem = driveSubsystem;
        m_translationXSupplier = translationXSupplier;
        m_translationYSupplier = translationYSupplier;
        m_rotationSupplier = rotationSupplier;
 
        addRequirements(m_driveSubsystem);
    }

    @Override
    public void execute() {
        ModifyAxis m_axisX = new ModifyAxis(m_translationXSupplier.getAsDouble(), 2);
        ModifyAxis m_axisY = new ModifyAxis(m_translationYSupplier.getAsDouble(), 2);
        ModifyAxis m_axisRot = new ModifyAxis(m_rotationSupplier.getAsDouble(), 2);

        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
        if(m_driverController.getRightBumper()){
            m_driveSubsystem.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    m_axisY.m_modifiedValue* 0.125,
                    m_axisX.m_modifiedValue* 0.125,
                    m_axisRot.m_modifiedValue * 0.0125,
                    m_driveSubsystem.getGyroscopeRotation()

                )
            );  
        }
        else if(m_driverController.getLeftBumper()){
            m_driveSubsystem.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    m_axisY.m_modifiedValue* 0.0625,
                    m_axisX.m_modifiedValue* 0.0625,
                    m_axisRot.m_modifiedValue * 0.0125,
                    m_driveSubsystem.getGyroscopeRotation()
                )
            );  
        }
        else{
            m_driveSubsystem.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    m_axisY.m_modifiedValue* 0.25,
                    m_axisX.m_modifiedValue * 0.25,
                    m_axisRot.m_modifiedValue * 0.025,
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
