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
    //Creates controller objects
    private final XBoxControllerEE m_driverController = new XBoxControllerEE(0);
    private final XBoxControllerEE m_operatorController = new XBoxControllerEE(1);

    double m_rotation, deltaX, errorX; 
  
    //Constructor for SwerveDrive
    /**
     * 
     * @param driveSubsystem drive Subsystem
     * @param translationXSupplier X Value
     * @param translationYSupplier Y value
     * @param rotationSupplier Rotation value
     */
    public SwerveDrive(DriveSubsystem driveSubsystem, DoubleSupplier translationXSupplier, DoubleSupplier translationYSupplier, DoubleSupplier rotationSupplier) {
        //sets local variable to member variables
        m_driveSubsystem = driveSubsystem;
        m_translationXSupplier = translationXSupplier;
        m_translationYSupplier = translationYSupplier;
        m_rotationSupplier = rotationSupplier;
 
        addRequirements(m_driveSubsystem);
    }

    @Override
    public void execute() {
        //modifys axis of controlls to ease control
        ModifyAxis m_axisX = new ModifyAxis(m_translationXSupplier.getAsDouble(), 2);
        ModifyAxis m_axisY = new ModifyAxis(m_translationYSupplier.getAsDouble(), 2);
        ModifyAxis m_axisRot = new ModifyAxis(m_rotationSupplier.getAsDouble(), 2);

        double vx = (m_axisX.m_modifiedValue) * m_driveSubsystem.getDriveSpeed();
        double vy = (m_axisY.m_modifiedValue) * m_driveSubsystem.getDriveSpeed();
        double vrot = (m_axisRot.m_modifiedValue) * m_driveSubsystem.getDriveSpeed();


        //drives robot at 50% speed
        if(m_driverController.getRightBumper()){
            m_driveSubsystem.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    vy* 0.125,
                    vx* 0.125,
                    vrot * 0.0125,
                    m_driveSubsystem.getGyroscopeRotation()

                )
            );  
        }
        //Drives robot at 25% speed
        else if(m_driverController.getLeftBumper() || m_operatorController.getDpadDown()){
            m_driveSubsystem.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    vy* 0.0625,
                    vx* 0.0625,
                    vrot * 0.0125,
                    m_driveSubsystem.getGyroscopeRotation()
                )
            );  
        }

        //Drive robot at full speed
        else{
            m_driveSubsystem.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    vy* 0.175, //0.25 normally
                    vx * 0.175, //""
                    vrot * 0.03, //0.03 normally
                    m_driveSubsystem.getGyroscopeRotation()
                )
            ); 
        }

    }

    @Override
    public void end(boolean interrupted) {
        //Stops drive base if command ends 
        m_driveSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}