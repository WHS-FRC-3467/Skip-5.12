package frc.robot.subsystems.Drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Feedback.Cameras.Limelight;

public class BasicLimelightAim extends CommandBase {
  // Initialize Variables
  DriveSubsystem m_drive;
  double m_rotation, m_targetThreshold, deltaX, error;
  boolean m_end;
  NetworkTableEntry tx, ty, ta;
  NetworkTable table;
  Limelight m_limelight;



  // Constructor for BasicLimelightAim
  public BasicLimelightAim(DriveSubsystem drive, Limelight limelight) {
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

    // tx = table.getEntry("tx");
    // ty = table.getEntry("ty");
    // ta = table.getEntry("ta");
  }

  @Override
  public void execute() {

    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");
    // get current X-axis target delta from center of image, in degrees.
    deltaX = tx.getDouble(0.0);
    System.out.println(deltaX);
    error = Math.abs(deltaX);

    // if(Math.abs(deltaX) >= 2)
    // {
    //   m_end = true;
    // }
    // If we are still outside our desired target range, rotate the robot.
    if (deltaX >= 2.0) {
      m_end = false;
      m_drive.drive(new ChassisSpeeds(0, 0, -deltaX));
    } 
    else if (deltaX <= -2.0){
      m_end = false;
      m_drive.drive(new ChassisSpeeds(0, 0, deltaX));
    }
    else{
      CommandScheduler.getInstance().cancel();
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
     if(error < 2){
       return true;
     }
     else{
       return false;
     }
  }
}