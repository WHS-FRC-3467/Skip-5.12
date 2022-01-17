// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drive;


import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;

public class AutoDrive extends CommandBase {
  DriveSubsystem m_drive;
  double m_angle, m_feet;
  double m_finalPosition, m_startEncoderValue;
  PIDController pidController = new PIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD);

  public AutoDrive(DriveSubsystem drive, double angle, double feet) {
    m_angle = angle;
    m_drive = drive;
    m_feet = feet;
  }

  @Override
  public void initialize() {
    m_finalPosition = m_drive.feetToEncoderAngle(m_feet);
    m_startEncoderValue = m_drive.getAverageEncoder();
    pidController.setTolerance(0.5);
  }

  @Override
  public void execute() {
    m_drive.setState(pidController.calculate(m_drive.getAverageEncoder()-m_startEncoderValue, m_finalPosition), m_angle);
    SmartDashboard.putNumber("Drive Distance", (m_drive.getAverageEncoder()-m_startEncoderValue) * (Math.PI * SdsModuleConfigurations.MK3_STANDARD.getWheelDiameter()/4096));
    SmartDashboard.putNumber("Drive error", pidController.getPositionError());
  }

  @Override
  public void end(boolean interrupted) {
    pidController.reset();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}