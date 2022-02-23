// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.Drive;

import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
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
  public BasicAutoDrive(DriveSubsystem drive, double angle, double meter, boolean forward) {
    //Set constructer variables equal to member variables 
    m_angle = angle;
    m_meter = meter;
    m_drive = drive;
    m_forward = forward;
    addRequirements(m_drive);
  }

  @Override
  public void initialize() {
    //Converts meter from constructer to encoder ticks
    m_finalPosition = m_drive.meterToEncoderTicks(m_meter);
    //Initializes start value
    m_startEncoderValue = m_drive.getAverageEncoder();
  }

  @Override
  public void execute() {
    // creates local variable current encoder
    double currentEncoder = m_drive.getAverageEncoder()-m_startEncoderValue;

    //Returns true if current encoder value is less than the goal point
    //If true sets drive base at a speed and angle 
    //If false sets drivebase to zero
    if(m_forward){
      if(currentEncoder <= m_finalPosition){
          m_drive.setState(DriveConstants.SimpleAutoVelocity, m_angle);
          m_end = false;
      }
      else{
        m_end = true;
      }
    }
    else if (m_forward == false){
      if(Math.abs(currentEncoder) <= m_finalPosition){
        m_drive.setState(-DriveConstants.SimpleAutoVelocity, m_angle);
        m_end=false;
      }
      else{
        m_end = true;
      }
    }
    

    //Puts drive distance and encoder distance to smart Dashboard
    SmartDashboard.putNumber("Drive Distance", (m_drive.getAverageEncoder()-m_startEncoderValue) * ((SdsModuleConfigurations.MK3_FAST.getDriveReduction() * SdsModuleConfigurations.MK3_FAST.getWheelDiameter() * Math.PI)/2048));
    SmartDashboard.putNumber("Encoder position", m_drive.getAverageEncoder()-m_startEncoderValue);
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