// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Control.XBoxControllerButton;
import frc.robot.Control.XBoxControllerDPad;
import frc.robot.Control.XBoxControllerEE;
import frc.robot.Feedback.LED.LEDDefault;
import frc.robot.Feedback.LED.LEDSubsystem;
import frc.robot.Subsystems.Climber.A0_CalibrateClimber;
import frc.robot.Subsystems.Climber.A1_PrepareToClimb;
import frc.robot.Subsystems.Climber.A9_DoItAll;
import frc.robot.Subsystems.Climber.AX_CancelClimb;
import frc.robot.Subsystems.Climber.ClimberSubsystem;
import frc.robot.Subsystems.Climber.MatchDefault;
import frc.robot.Subsystems.Drive.DriveSubsystem;
import frc.robot.Subsystems.Drive.SwerveDrive;
import frc.robot.Subsystems.Intake.DriveIntake;
import frc.robot.Subsystems.Intake.IntakeSubsystem;
import frc.robot.Subsystems.Shooter.AutoShoot;
import frc.robot.Subsystems.Shooter.Shoot;
import frc.robot.Subsystems.Shooter.ShooterSubsystem;
import frc.robot.Subsystems.Tower.DriveTower;
import frc.robot.Subsystems.Tower.TowerSubsystem;


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
  private final LEDSubsystem m_ledSubsystem = new LEDSubsystem();

  private final XBoxControllerEE m_controller = new XBoxControllerEE(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {  
    
    new Pneumactics();

    // Detect if controllers are missing / Stop multiple warnings
    DriverStation.silenceJoystickConnectionWarning(DriveConstants.PRACTICE);
    

    // Comment out for simulation

    SmartDashboard.putData(new A0_CalibrateClimber(m_climberSubsystem));

    // Configure the button bindings
    configureButtonBindings();
    
    
    m_driveSubsystem.setDefaultCommand(new SwerveDrive(m_driveSubsystem, 
                                      () -> -((m_controller.getLeftX())) * DriveSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
                                      () -> -((m_controller.getLeftY())) * DriveSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
                                      () -> -((m_controller.getRightX())) * DriveSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND));

    m_intakeSubsystem.setDefaultCommand(new DriveIntake(m_intakeSubsystem,
                                        () -> (m_controller.getRightTriggerAxis()),  
                                        () -> (m_controller.getLeftTriggerAxis()), 
                                        m_towerSubsystem));

    m_towerSubsystem.setDefaultCommand(new DriveTower(m_towerSubsystem,
                                        () -> -m_controller.getLeftY()));

    m_ledSubsystem.setDefaultCommand(new LEDDefault(m_ledSubsystem, m_towerSubsystem));
    
    m_climberSubsystem.setDefaultCommand(new MatchDefault(m_climberSubsystem));

    // Make the Climb Sequence commands available on SmartDash
    // SmartDashboard.putData("Calibrate Climber", new A0_CalibrateClimber(m_climberSubsystem));
    // SmartDashboard.putData(new A2_LiftAndReach(m_climberSubsystem));
    // SmartDashboard.putData(new A3_HookAndReach(m_climberSubsystem));
    // SmartDashboard.putData(new A4_HookAndStop(m_climberSubsystem));
    // SmartDashboard.putData(new A9_DoItAll(m_climberSubsystem));
    // SmartDashboard.putData(new AX_CancelClimb(m_climberSubsystem));
    // }

    // if(Constants.tuningMode){
    //   SmartDashboard.putData("Test Shooter", new TestShoot(m_towerSubsystem, m_shooterSubystem));
    // }
  } 

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    //Driver Controller
      
    // Back button zeros the gyroscope
    new XBoxControllerDPad(m_controller, XBoxControllerEE.DPad.kDPadUp)
        .whenActive(m_driveSubsystem::zeroGyroscope, m_driveSubsystem);

    //Shoot lower hub
    new XBoxControllerButton(m_controller, XBoxControllerEE.Button.kA)
      .whenHeld(new AutoShoot(m_shooterSubystem, m_towerSubsystem, ShooterConstants.kLowerHubVelocity + 100, ShooterConstants.kShooterGains, Value.kForward));

    //Shoot Upper Hub
    new XBoxControllerButton(m_controller, XBoxControllerEE.Button.kB)
      .whenHeld(new AutoShoot(m_shooterSubystem, m_towerSubsystem, ShooterConstants.kUpperHubFenderVelocity, ShooterConstants.kShooterGains, Value.kReverse));

    //Shoot Tarmac
    new XBoxControllerButton(m_controller, XBoxControllerEE.Button.kY)
      .whenHeld(new AutoShoot(m_shooterSubystem, m_towerSubsystem, ShooterConstants.kTarmacVelocity, ShooterConstants.kShooterGains, Value.kForward));

    //Shoot Ranging
    new XBoxControllerButton(m_controller, XBoxControllerEE.Button.kX)
      .whenHeld(new Shoot(m_shooterSubystem, ShooterConstants.kLaunchpadVelocity, Value.kForward));



    new XBoxControllerButton(m_controller, XBoxControllerEE.Button.kBack)
      .whenPressed(new A1_PrepareToClimb(m_climberSubsystem));

    new XBoxControllerButton(m_controller, XBoxControllerEE.Button.kStart)
      .whenPressed(new A9_DoItAll(m_climberSubsystem));

    new XBoxControllerButton(m_controller, XBoxControllerEE.Button.kLeftBumper)
      .whileHeld(new RunCommand(()->m_towerSubsystem.driveWholeTower(1.0)).andThen(new RunCommand(()->m_towerSubsystem.driveWholeTower(0.0))));

    new XBoxControllerButton(m_controller, XBoxControllerEE.Button.kRightBumper)
      .whileHeld(new RunCommand(()->m_towerSubsystem.driveWholeTower(-1.0)).andThen(new RunCommand(()->m_towerSubsystem.driveWholeTower(0.0))));

    new XBoxControllerDPad(m_controller, XBoxControllerEE.DPad.kDPadLeft)
      .whenActive(new AX_CancelClimb(m_climberSubsystem));

    new XBoxControllerDPad(m_controller, XBoxControllerEE.DPad.kDPadDown)
      .whenActive(new AX_CancelClimb(m_climberSubsystem));

    new XBoxControllerDPad(m_controller, XBoxControllerEE.DPad.kDPadRight)
      .whenActive(new AX_CancelClimb(m_climberSubsystem));

  }

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
