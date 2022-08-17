package frc.robot.Subsystems.Drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Control.XBoxControllerEE;
import frc.robot.Feedback.Cameras.LimelightSubsystem;
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
    NetworkTable table;
    NetworkTableEntry tx;
  
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

        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
        //drives robot at 50% speed
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
        //Drives robot at 25% speed
        else if(m_driverController.getLeftBumper() || m_operatorController.getDpadDown()){
            m_driveSubsystem.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    m_axisY.m_modifiedValue* 0.0625,
                    m_axisX.m_modifiedValue* 0.0625,
                    m_axisRot.m_modifiedValue * 0.0125,
                    m_driveSubsystem.getGyroscopeRotation()
                )
            );  
        }
        else if (m_driverController.getAButton()){
            table = NetworkTableInstance.getDefault().getTable("limelight");

            //Turns on 
            LimelightSubsystem.setVisionMode();
                    
            //Updates network table variables
            tx = table.getEntry("tx");

            // get current X-axis target delta from center of image, in degrees.
            deltaX = tx.getDouble(0.0);
            
            //gets absolute distance from the center of the target.
            errorX = Math.abs(deltaX);
        
            m_rotation = Math.max(-1.5, Math.min(1.5, deltaX/6.0));

            if(m_rotation < 0.5 && m_rotation > 0){
                m_rotation = 0.5;
            }
            else if( m_rotation > -0.5 && m_rotation < 0.0){
                m_rotation = -0.5;
            }
            if(errorX<1){
                m_driveSubsystem.drive(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                        m_axisY.m_modifiedValue* 0.125, 
                        m_axisX.m_modifiedValue * 0.125, 
                        0, 
                        m_driveSubsystem.getGyroscopeRotation()
                    )
                ); 
            }
            else{
                m_driveSubsystem.drive(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                        m_axisY.m_modifiedValue* 0.125, 
                        m_axisX.m_modifiedValue * 0.125, 
                        -m_rotation, 
                        m_driveSubsystem.getGyroscopeRotation()
                    )
                ); 

            }
        }
        //Drive robot at full speed
        else{
            m_driveSubsystem.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    m_axisY.m_modifiedValue* 0.175, //0.25 normally
                    m_axisX.m_modifiedValue * 0.175, //""
                    m_axisRot.m_modifiedValue * 0.03, //0.03 normally
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