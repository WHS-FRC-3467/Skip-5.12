// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Autonomous.OneBallAuto;
import frc.robot.Autonomous.TestAuto;
import frc.robot.Constants.ShooterConstants;
//import frc.robot.Control.XBoxControllerDPad;
import frc.robot.Control.XboxControllerButton;
import frc.robot.Control.XboxControllerEE;
import frc.robot.Feedback.Cameras.Limelight;
import frc.robot.subsystems.Climber.ClimberSubsystem;
import frc.robot.subsystems.Climber.ExtendClimber;
import frc.robot.subsystems.Drive.DriveSubsystem;
import frc.robot.subsystems.Drive.SwerveDrive;
import frc.robot.subsystems.Intake.DriveIntake;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Intake.ToggleIntake;
import frc.robot.subsystems.Shooter.AutoShoot;
import frc.robot.subsystems.Shooter.ShootLowerHub;
import frc.robot.subsystems.Shooter.ShootUpperHub;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.Tower.DriveTower;
import frc.robot.subsystems.Tower.TowerSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */

public class RobotContainer {
  
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();
  private final ShooterSubsystem m_shooterSubystem = new ShooterSubsystem();
  private final TowerSubsystem m_towerSubsystem = new TowerSubsystem();
  private final XboxControllerEE m_driverController = new XboxControllerEE(0);
  private final XboxControllerEE m_operatorController = new XboxControllerEE(1);

  private final TestAuto m_testAuto = new TestAuto(m_driveSubsystem);
  private final OneBallAuto m_oneBallAuto = new OneBallAuto(m_driveSubsystem, m_shooterSubystem, m_towerSubsystem);

  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    new Pneumactics();
    m_chooser.addOption("Test Auto", m_testAuto);
    m_chooser.addOption("One Ball Auto", m_oneBallAuto);
  
    SmartDashboard.putData(m_chooser);

    Limelight.initialize();
    Limelight.setDriverMode();

    // Configure the button bindings
    configureButtonBindings();
    m_driveSubsystem.setDefaultCommand(new SwerveDrive(m_driveSubsystem, 
                                      () -> (m_driverController.getLeftX()) * DriveSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
                                      () -> -(m_driverController.getLeftY()) * DriveSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
                                      () -> (m_driverController.getRightX()) * DriveSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND));
    
    m_intakeSubsystem.setDefaultCommand(new DriveIntake(m_intakeSubsystem,
                                        () -> (m_driverController.getRightTriggerAxis()),  
                                        () -> (m_driverController.getLeftTriggerAxis())));

    m_climberSubsystem.setDefaultCommand(new ExtendClimber(m_climberSubsystem, 
                                        () -> -m_operatorController.getRightY()));
  
    m_towerSubsystem.setDefaultCommand(new DriveTower(m_towerSubsystem,  
                                      () -> -m_operatorController.getLeftY()));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    //Driver Controller
    new XboxControllerButton(m_driverController, XboxControllerEE.Button.kLeftBumper)
    .whenPressed(new InstantCommand(m_intakeSubsystem::intakeDeploy, m_intakeSubsystem));
    
    new XboxControllerButton(m_driverController, XboxControllerEE.Button.kRightBumper)
    .whenPressed(new InstantCommand(m_intakeSubsystem::intakeRetract, m_intakeSubsystem));

    new XboxControllerButton(m_driverController, XboxControllerEE.Button.kBack)
    .whenHeld(new InstantCommand(m_driveSubsystem::zeroGyroscope, m_driveSubsystem));

    new XboxControllerButton(m_driverController, XboxControllerEE.Button.kA)
    .whenHeld(new ToggleIntake(m_intakeSubsystem));


    //Operator controller    
    new XboxControllerButton(m_operatorController, XboxControllerEE.Button.kA)
    .whenHeld(new ShootLowerHub(m_shooterSubystem, m_towerSubsystem));

    new XboxControllerButton(m_operatorController, XboxControllerEE.Button.kB)
    .whenHeld(new ShootUpperHub(m_shooterSubystem, m_towerSubsystem));

    new XboxControllerButton(m_operatorController, XboxControllerEE.Button.kX)
    .whenHeld(new AutoShoot(m_shooterSubystem, m_towerSubsystem, ShooterConstants.upperHubVelocity));


    new XboxControllerButton(m_operatorController, XboxControllerEE.Button.kLeftBumper)
    .whileActiveContinuous(new InstantCommand(m_shooterSubystem::deployHood, m_shooterSubystem));
    
    new XboxControllerButton(m_operatorController, XboxControllerEE.Button.kRightBumper)
    .whileActiveContinuous(new InstantCommand(m_shooterSubystem::retractHood, m_shooterSubystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_testAuto;
  }
}
