package frc.robot.subsystems.Drive;


import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class BasicAutoDrive extends CommandBase {
  //Initialize Variables
  DriveSubsystem m_drive;
  double m_angle, m_meter, m_XTranslation, m_YTranslation;
  double m_finalPosition;
  boolean m_forward;
  boolean m_end;
  boolean m_reverse;
  //Constructor for BasicAutoDrive
  public BasicAutoDrive(DriveSubsystem drive, double angle, double meter, double xTranslation, double yTranslation) {
    //Set constructer objects equal to member variables 
    m_angle = angle;
    m_meter = meter;
    m_drive = drive;

    m_YTranslation = yTranslation;
    m_XTranslation = xTranslation;
    addRequirements(m_drive);
  }

  @Override
  public void initialize() {
    m_finalPosition = m_drive.meterToEncoderTicks(m_meter);
    //Converts meter from constructer to encoder ticks
    m_drive.resetDriveEncoders();
    System.out.println("start");
  }

  @Override
  public void execute() {

    if(Math.abs(m_drive.getAverageEncoder()) <= Math.abs(m_finalPosition)){
      m_drive.drive(
        
        ChassisSpeeds.fromFieldRelativeSpeeds(
            m_XTranslation,
            m_YTranslation,
            0.0,
            m_drive.getGyroscopeRotation()
        )
    );      m_end = false;
    }
    else{
      m_drive.setState(0.0, 0.0);
      m_end = true;
    }
      
      System.out.println(m_finalPosition);
      System.out.println(m_drive.getAverageEncoder());

    double driveDistance = m_drive.encoderTicksToMeter(m_drive.getAverageEncoder());
    //Puts drive distance and encoder distance to smart Dashboard
    SmartDashboard.putNumber("Drive Distance", driveDistance);
    //SmartDashboard.putNumber("Encoder position", m_drive.getAverageEncoder());
  }

  
  @Override
  public void end(boolean interrupted) {
    //When finished the drivebase is set to zero
    m_drive.setState(0.0, 0.0);
    System.out.println("end");
  }

  @Override
  public boolean isFinished() {
    return m_end;
  }
}