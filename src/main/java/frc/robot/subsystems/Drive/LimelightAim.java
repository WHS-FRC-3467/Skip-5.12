package frc.robot.subsystems.Drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Feedback.Cameras.Limelight;

public class LimelightAim extends CommandBase {
  // Initialize Variables
  DriveSubsystem m_drive;
  double m_rotation, m_translationY, m_targetThreshold, deltaX, deltaY, deltaYTargetTarmac, errorX, errorY, count; 
  boolean m_end;
  NetworkTableEntry tx, ty, ta;
  NetworkTable table;
  Limelight m_limelight;


  // Constructor for BasicLimelightAim
  public LimelightAim(DriveSubsystem drive, Limelight limelight) {
    m_limelight = limelight;
    m_drive = drive;
    addRequirements(m_drive, m_limelight);
  }

  @Override
  public void initialize() {
    System.out.println("start BasicLimelightAim");

    // Initialize parent NetworkTable
    table = NetworkTableInstance.getDefault().getTable("limelight");

    // enable limelight LEDs
    // table.getEntry("ledMode").setNumber(3);


    Limelight.setVisionMode();

    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");
    count = 0;

    deltaYTargetTarmac = 0;

  }

  @Override
  public void execute() {

    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
	  //removed ta since it's unused and slows the loop
	
    // get current X-axis target delta from center of image, in degrees.
    deltaX = tx.getDouble(0.0);
	
	// get current Y-axis target delta from center of image, with the offset in account, in degrees.
	  deltaY = ty.getDouble(0.0) - deltaYTargetTarmac; 
	
	  errorX = Math.abs(deltaX);
	  errorY = Math.abs(deltaY);

  //fixed clamp logic here
	//increased limits to 1.5 now that we are slowing down approaching target
	//feel free to lower these, or divide by a bigger number if oscillations occur
    m_rotation = Math.max(-1.5, Math.min(1.5, deltaX/4.0));

	//invert the driving direction with -deltaY, since: 
	//target at top of frame = deltaY positive = drive back (negative)
	  m_translationY = Math.max(-1.5, Math.min(1.5, (-deltaY)/4.0)); 

    m_drive.drive(new ChassisSpeeds(-m_translationY, 0, -m_rotation));
	
	//tightened tolerances to +/- 1 degree now that we are 
	//slowing down as the robot is approaching the target
    if(errorX < 1.0 && errorY < 1.0){
      System.out.println("End command");
      m_end = true;

      m_drive.drive(new ChassisSpeeds(0, 0, 0));
    }
  }


  @Override
  public void end(boolean interrupted) {

    m_drive.drive(new ChassisSpeeds(0, 0, 0));

    System.out.println("end BasicLimelightAim");
    Limelight.setDriverMode();
  }

  @Override
  public boolean isFinished() {
    count++;
    if(count > 100){
      return m_end;
    }
    else{
      return false;
    }
  }
}