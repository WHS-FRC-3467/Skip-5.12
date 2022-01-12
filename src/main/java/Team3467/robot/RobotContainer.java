// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package Team3467.robot;

import Team3467.robot.Control.XboxControllerEE;
import Team3467.robot.subsystems.Drive.DriveSubsystem;
import Team3467.robot.subsystems.Drive.SwerveDrive;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();

  private final XboxControllerEE m_controller = new XboxControllerEE(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    m_driveSubsystem.setDefaultCommand(new SwerveDrive(m_driveSubsystem, 
                                      () -> -(m_controller.getLeftY()) * DriveSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
                                      () -> -(m_controller.getLeftX()) * DriveSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
                                      () -> -(m_controller.getRightX()) * DriveSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
