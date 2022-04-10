package frc.robot.subsystems.Drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Feedback.Cameras.Limelight;

public class BasicLimelightAim extends CommandBase {
  // Initialize Variables
  DriveSubsystem m_drive;
  double m_rotation, m_targetThreshold;
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
    table.getEntry("ledMode").setNumber(3);

    Limelight.setVisionMode();

    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");

  }

  @Override
  public void execute() {
    table.getEntry("ledMode").setNumber(3);

    // initialize NetworkTableEntries. If these values do not update, try
    // moving them to execute().
    // get current X-axis target delta from center of image, in degrees.
    double deltaX = tx.getDouble(0.0);

    // calculate rotational velocity. 
    m_rotation = Math.toRadians(deltaX)*DriveConstants.LIMELIGHT_X_P;
    
    // limit minimum and maximum rotational velocity.
    m_rotation = Math.min(m_rotation, DriveConstants.LIMELIGHT_X_VELOCITY_LIMIT);
    m_rotation = Math.max(m_rotation, -DriveConstants.LIMELIGHT_X_VELOCITY_LIMIT);

    // If we are still outside our desired target range, rotate the robot.
    if (Math.abs(deltaX) >= DriveConstants.LIMELIGHT_X_TOLERANCE) {
      m_drive.drive(
          ChassisSpeeds.fromFieldRelativeSpeeds(
              0,
              0,
              m_rotation,
              m_drive.getGyroscopeRotation()));
    } else {
      ChassisSpeeds.fromFieldRelativeSpeeds(
          0.0,
          0.0,
          0.0,
          m_drive.getGyroscopeRotation());
    }

    // print diagnostic information
    System.out.println("Limelight X delta in degrees: " + deltaX
        + "Current rotational speed in radians/sec: " + m_rotation);
    SmartDashboard.putNumber("Limelight X-Delta", deltaX);

  }

  @Override
  public void end(boolean interrupted) {

    // When finished the drivebase is set to zero speed
    ChassisSpeeds.fromFieldRelativeSpeeds(
        0.0,
        0.0,
        0.0,
        m_drive.getGyroscopeRotation());
    //disable limelight LEDs
    table.getEntry("ledMode").setNumber(1);
    System.out.println("end BasicLimelightAim");
    Limelight.setDriverMode();
  }

  @Override
  public boolean isFinished() {
    double deltaX = tx.getDouble(0.0);

    if (Math.abs(deltaX) >= DriveConstants.LIMELIGHT_X_TOLERANCE){
      return false;
    }
    else{
      return true;
    }

  }
}