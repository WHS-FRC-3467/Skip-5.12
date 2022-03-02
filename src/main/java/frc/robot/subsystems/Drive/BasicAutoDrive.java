package frc.robot.subsystems.Drive;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;

public class BasicAutoDrive extends CommandBase {
  //Initialize Variables
  DriveSubsystem m_drive;
  double m_angle, m_meter;
  double m_finalPosition, m_startEncoderValue;
  boolean m_forward;
  boolean m_end;
  //Constructor for BasicAutoDrive
  public BasicAutoDrive(DriveSubsystem drive, double angle, double meter) {
    //Set constructer variables equal to member variables 
    m_angle = angle;
    m_meter = meter;
    m_drive = drive;
    addRequirements(m_drive);
  }
  //drives robot oriented
  // intake is reverse 
  // shooter is forward

  @Override
  public void initialize() {
    m_finalPosition = m_drive.meterToEncoderTicks(m_meter);
    //Converts meter from constructer to encoder ticks
    m_drive.resetDriveEncoders();
    if(m_finalPosition > 0){
      m_forward = true;
    }
    if(m_finalPosition <0 ){
      m_forward = false;
    }
  }

  @Override
  public void execute() {
      if(m_forward == true){
        if(Math.abs(m_drive.getAverageEncoder()) <= Math.abs(m_finalPosition)){
          m_drive.setState(DriveConstants.SimpleAutoVelocity, m_angle);
          m_end = false;
        }
        else{
          m_drive.setState(0.0, 0.0);
          m_end = true;
        }
      }
      
      if(m_forward == false){
        if( m_drive.getAverageEncoder() >= m_finalPosition){
          m_drive.setState(-DriveConstants.SimpleAutoVelocity, m_angle);
          m_end = false;
        }    
        else{
          m_drive.setState(0.0, 0.0);
          m_end = true;
        }
      }
     
        
      
    double driveDistance = m_drive.encoderTicksToMeter(m_drive.getAverageEncoder());
    //Puts drive distance and encoder distance to smart Dashboard
    SmartDashboard.putNumber("Drive Distance", driveDistance);
    SmartDashboard.putNumber("Encoder position", m_drive.getAverageEncoder());
  }

  
  @Override
  public void end(boolean interrupted) {
    //When finished the drivebase is set to zero
    m_drive.setState(0.0, 0.0);
  }

  @Override
  public boolean isFinished() {
    return m_end;
  }
}
