// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autonomous;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Subsystems.Drive.BasicAutoDrive;
import frc.robot.Subsystems.Drive.DriveSubsystem;
import frc.robot.Subsystems.Shooter.AutoShoot;
import frc.robot.Subsystems.Shooter.ShooterSubsystem;
import frc.robot.Subsystems.Tower.TowerSubsystem;

public class RightSideOneBall extends SequentialCommandGroup {
  //import subsystem
  DriveSubsystem m_drive;
  TowerSubsystem m_tower;
  ShooterSubsystem m_shooter;

  /**
   * Constructor for RightSideOneBall
   * @param drive Drive Subsystem
   * @param shooter Shooter Subsystem
   * @param tower Tower subsystem
   */
  public RightSideOneBall(DriveSubsystem drive, ShooterSubsystem shooter, TowerSubsystem tower) {
    //Set local variables to member variables
    m_drive = drive;
    m_tower = tower;
    m_shooter = shooter;

    addCommands(
      //Reset drive encoders to zero distance
      new InstantCommand(m_drive::resetDriveEncoders),
      //Shoot one ball
      new AutoShoot(m_shooter, m_tower, ShooterConstants.kUpperHubFenderVelocity, ShooterConstants.kUpperHubFenderGains, Value.kReverse),
      //Drive away
      new BasicAutoDrive(drive, 3, 0.5, -0.5, 0.0)
    );
  }
}
