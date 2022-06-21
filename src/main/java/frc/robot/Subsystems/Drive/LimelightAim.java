package frc.robot.Subsystems.Drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Feedback.Cameras.LimelightSubsystem;

public class LimelightAim extends CommandBase {
  // Initialize Variables
  DriveSubsystem m_drive;
  double m_rotation, m_translationY, m_targetThreshold, deltaX, deltaY, deltaYTargetTarmac, errorX, errorY, count; 
  boolean m_end, m_waitForTarget, m_hasTarget, m_ranging;
  NetworkTableEntry tx, ty, ta, tv;
  NetworkTable table;
  LimelightSubsystem m_limelight;


  public LimelightAim(DriveSubsystem drive, LimelightSubsystem limelight) {
    m_limelight = limelight;
    m_drive = drive;
    m_waitForTarget = false;
    m_ranging = true;
    addRequirements(m_drive, m_limelight);
  }
  /** 
  * @param drive Drive Subsystem
  * @param limelight Limelight 
  * @param waitForTarget The command ends if there is no target
  * @param ranging Driving to the distance for tarmac shot
  */
  public LimelightAim(DriveSubsystem drive, LimelightSubsystem limelight, boolean waitForTarget, boolean ranging){
    m_limelight = limelight;
    m_drive = drive;
    m_waitForTarget = waitForTarget;
    m_ranging = ranging;
    addRequirements(m_drive, m_limelight);
  }

  @Override
  public void initialize() {
    System.out.println("start LimelightAim");

    // Initialize parent NetworkTable
    table = NetworkTableInstance.getDefault().getTable("limelight");
    
    //Puts limelight into threshold mode where it can see the goal
    //Turns on LEDs
    LimelightSubsystem.setVisionMode();

    //gets limelight network table entries
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    tv = table.getEntry("tv");
    count = 0;

    //Sets the place in limelight where goal needs to be
    // Positive is up/forward, Negative is down/down
    deltaYTargetTarmac = 5.0;

  }

  @Override
  public void execute() {
    //Updates network table variables
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    tv = table.getEntry("tv");
    m_hasTarget = tv.getDouble(1.0) == 1.0;
	  //removed ta since it's unused and slows the loop
	
    // get current X-axis target delta from center of image, in degrees.
    deltaX = tx.getDouble(0.0);
	
  	// get current Y-axis target delta from center of image, with the offset in account, in degrees.
	  deltaY = ty.getDouble(0.0) - deltaYTargetTarmac; 
    
    //gets absolute distance from the center of the target.
	  errorX = Math.abs(deltaX);
	  
    if(m_ranging == true) {
      errorY = Math.abs(deltaY);
    }
    else {
      errorY = 0.0;
    }
    //fixed clamp logic here
    //increased limits to 1.5 now that we are slowing down approaching target
    //feel free to lower these, or divide by a bigger number if oscillations occur
    m_rotation = Math.max(-1.5, Math.min(1.5, deltaX/6.0));

	  //invert the driving direction with -deltaY, since: 
  	//target at top of frame = deltaY positive = drive back (negative)
	  m_translationY = Math.max(-1.5, Math.min(1.5, (-deltaY)/4.0)); 

    if(m_ranging){
      m_drive.drive(new ChassisSpeeds(-m_translationY, 0, -m_rotation));
    } else {
      m_drive.drive(new ChassisSpeeds(0, 0, -m_rotation));
    }
	
    //tightened tolerances to +/- 1 degree now that we are 
    //slowing down as the robot is approaching the target
    if((errorX < 0.5 && errorY < 1.0) && (!m_waitForTarget || m_hasTarget)){
      System.out.println("End command");
      m_end = true;

      m_drive.drive(new ChassisSpeeds(0, 0, 0));
    }
  }


  @Override
  public void end(boolean interrupted) {

    m_drive.drive(new ChassisSpeeds(0, 0, 0));

    System.out.println("end LimelightAim");
    LimelightSubsystem.setDriverMode();
  }

  @Override
  public boolean isFinished() {
    count++;
    //if command finishing behavior is inconsistent, remove "|| m_hasTarget"
    if(count > 50){ 
      return m_end;
    }
    else{
      return false;
    }
  }
}
