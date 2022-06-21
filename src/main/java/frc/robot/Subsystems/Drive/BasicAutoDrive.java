package frc.robot.Subsystems.Drive;


import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class BasicAutoDrive extends CommandBase {
  //Initialize Variables and import subsystems
  DriveSubsystem m_drive;
  double m_angle, m_meter, m_XTranslation, m_YTranslation, m_rotation;
  double m_finalPosition;
  boolean m_forward;
  boolean m_end;
  boolean m_reverse;


  //Constructor for BasicAutoDrive
  /**
   * 
   * @param drive Drive subsystem
   * @param meter The distance driven
   * @param xTranslation X meters per second
   * @param yTranslation Y meters per second
   * @param rotation Rotation in radians per second
   */
  public BasicAutoDrive(DriveSubsystem drive, double meter, double xTranslation, double yTranslation, double rotation) {
    //Set constructer objects equal to member variables 
    m_meter = meter;
    m_drive = drive;

    m_YTranslation = yTranslation;
    m_XTranslation = xTranslation;
    m_rotation = rotation;
    //Requires drive subsytem
    addRequirements(m_drive);
  }

  @Override
  public void initialize() {
    //Converts meter from constructer to encoder ticks
    m_finalPosition = m_drive.meterToEncoderTicks(m_meter);
    //set drive encoders to zero
    m_drive.resetDriveEncoders();

    System.out.println("start");
  }

  @Override
  public void execute() {
    //If the distance traveled is less than the distance needed to go then it will drive at the direction and velocoty from the construtor
    //else it will end the command
    if(Math.abs(m_drive.getAverageEncoder()) <= Math.abs(m_finalPosition)){
      m_drive.drive(
        //Drive robot at constructor speeds
        ChassisSpeeds.fromFieldRelativeSpeeds(
            -m_XTranslation,
            -m_YTranslation,
            m_rotation,
            m_drive.getGyroscopeRotation()
        )
      );      
      //Do not end command
      m_end = false;
    }
    else{
      //Stop drivebase and end command
      m_drive.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
      m_end = true;
    }
    
    //prints goal distance and current distance traveled
    System.out.println(m_finalPosition);
    System.out.println(m_drive.getAverageEncoder());

    double driveDistance = m_drive.encoderTicksToMeter(m_drive.getAverageEncoder());
    //Puts drive distance and encoder distance to smart Dashboard
    SmartDashboard.putNumber("Drive Distance", driveDistance);

  }

  
  @Override
  public void end(boolean interrupted) {
    //When finished the drivebase is set to zero
    m_drive.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    System.out.println("end");
  }

  @Override
  public boolean isFinished() {
    //ends command if m_end is true
    return m_end;
  }
}
