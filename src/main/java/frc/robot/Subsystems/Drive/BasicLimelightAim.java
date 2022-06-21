package frc.robot.Subsystems.Drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Feedback.Cameras.LimelightSubsystem;

public class BasicLimelightAim extends CommandBase {
  // Initialize Variables
  DriveSubsystem m_drive;
  double m_rotation, m_targetThreshold, deltaX, error, count;
  boolean m_end;
  NetworkTableEntry tx, ty, ta;
  NetworkTable table;
  LimelightSubsystem m_limelight;

  // Constructor for BasicLimelightAim
  /**
   * 
   * @param drive Drive subsystem
   * @param limelight Limelight subsystem
   */
  public BasicLimelightAim(DriveSubsystem drive, LimelightSubsystem limelight) {
    //sets local variables to memeber variables
    m_limelight = limelight;
    m_drive = drive;
    //Requires drive and limelight
    addRequirements(m_drive, m_limelight);
  }

  @Override
  public void initialize() {
    System.out.println("start BasicLimelightAim");

    // Initialize parent NetworkTable
    table = NetworkTableInstance.getDefault().getTable("limelight");

    // enable limelight LEDs
    // table.getEntry("ledMode").setNumber(3);

    //Puts limelight into threshold mode where it can see the goal
    //Turns on LEDs
    LimelightSubsystem.setVisionMode();

    //gets limelight network table entries
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");

    count = 0;
  }

  @Override
  public void execute() {
    //Updates local variables to current limelight table entries
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");

    // get current X-axis target delta from center of image, in degrees.
    deltaX = tx.getDouble(0.0);
    System.out.println(deltaX);
    error = Math.abs(deltaX);

    //Creates velocity for drive base using deltaX
    m_rotation = deltaX/2;
    m_rotation = Math.max(m_rotation, 1.0);
    m_rotation = Math.min(m_rotation, -1.0);

    //if delta X is greater than 1 then it will turn right towards goal 
    //command won't end
    if (deltaX >= 1.0) {
      m_end = false;
      //sets drive base to speed determined by deltaX
      m_drive.drive(new ChassisSpeeds(0, 0, m_rotation));
    } 
    //if delta X is less than 1 then it will turn left towards goal 
    //command won't end
    else if (deltaX <= -1.0){
      m_end = false;      
      //sets drive base to speed determined by deltaX
      m_drive.drive(new ChassisSpeeds(0, 0, -m_rotation));
    }
    //If delta x is within tollerance command will end
    else{
      System.out.println("End command");
      //ends command
      m_end = true;
      //Stops drive base
      m_drive.drive(new ChassisSpeeds(0, 0, 0));
    }
  }


  @Override
  public void end(boolean interrupted) {
    //Stops drive base
    m_drive.drive(new ChassisSpeeds(0, 0, 0));

    System.out.println("end BasicLimelightAim");
    //Puts limelight back into color vision and turns of LEDS
    LimelightSubsystem.setDriverMode();
  }

  @Override
  public boolean isFinished() {
    System.out.println(error);
    //Increases count of time command has run by 1
    //Loop runs every 0.02 seconds
    count++;
    //Won't end before command has been going for 1 second
    if(count > 50){
      //ends command
      return m_end;
    }
    else{
      //does not end command
      return false;
    }
  }
}
