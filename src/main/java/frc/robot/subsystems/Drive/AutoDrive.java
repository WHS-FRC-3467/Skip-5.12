// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drive;


import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class AutoDrive extends CommandBase {
  DriveSubsystem m_drive;
  double m_angle, m_meter;
  double m_finalPosition, m_startEncoderValue;
  PIDController m_pidController = new PIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD);

  public AutoDrive(DriveSubsystem drive, double angle, double meter) {
    m_angle = angle;
    m_meter = meter;
    m_drive = drive;
    addRequirements(m_drive);
  }

  @Override
  public void initialize() {
    m_finalPosition = m_drive.meterToEncoderTicks(m_meter);
    m_startEncoderValue = m_drive.getAverageEncoder();
    m_pidController.setSetpoint(m_finalPosition);
    m_pidController.setTolerance(DriveConstants.driveTollerance);
  }

  @Override
  public void execute() {
    if(m_pidController.atSetpoint()){
      m_drive.setState(0.0, m_angle);
    }
    m_drive.setState(m_pidController.calculate(m_drive.getAverageEncoder()-m_startEncoderValue), m_angle);
    SmartDashboard.putNumber("Drive Distance", (m_drive.getAverageEncoder()-m_startEncoderValue) * (Math.PI * SdsModuleConfigurations.MK3_FAST.getWheelDiameter()/2048));
    SmartDashboard.putNumber("Drive error", m_pidController.getPositionError());
    SmartDashboard.putNumber("Encoder position", m_drive.getAverageEncoder());
  }

  @Override
  public void end(boolean interrupted) {
    m_pidController.reset();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}