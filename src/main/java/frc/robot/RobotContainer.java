// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Autonomous.OneBallAuto;
import frc.robot.Autonomous.TwoBallAuto;
import frc.robot.Constants.DriveConstants;
import frc.robot.Control.XBoxControllerDPad;
import frc.robot.Control.XBoxControllerButton;
import frc.robot.Control.XBoxControllerEE;
import frc.robot.Feedback.Cameras.Limelight;
import frc.robot.subsystems.Climber.A0_CalibrateClimber;
import frc.robot.subsystems.Climber.A1_PrepareToClimb;
import frc.robot.subsystems.Climber.A2_LiftToBar;
import frc.robot.subsystems.Climber.A3_ReachToNextBar;
import frc.robot.subsystems.Climber.A4_HookToNextBar;
import frc.robot.subsystems.Climber.ClimberSubsystem;
import frc.robot.subsystems.Climber.MagicClimbByDash;
import frc.robot.subsystems.Climber.MagicClimbByStick;
import frc.robot.subsystems.Climber.ManualClimbByStick;
import frc.robot.subsystems.Drive.DriveSubsystem;
import frc.robot.subsystems.Drive.SwerveDrive;
import frc.robot.subsystems.Intake.DriveIntake;
import frc.robot.subsystems.Intake.IntakeOverride;
import frc.robot.subsystems.Intake.IntakeSubsystem;
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

  private final XBoxControllerEE m_driverController = new XBoxControllerEE(0);
  private final XBoxControllerEE m_operatorController = new XBoxControllerEE(1);

  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {    
    new Pneumactics();
  
    // Detect if controllers are missing / Stop multiple warnings
    DriverStation.silenceJoystickConnectionWarning(DriveConstants.PRACTICE);
	
	// Setup the Swerve Subsystem (w/ simulation support)
  	// dt = BearSwerveHelper.createBearSwerve();
    // m_swerveSubsystem = BearSwerveHelper.createSwerveSubsystem(dt);

    // Populate Auto Chooser
    Shuffleboard.getTab("Driver Dash").add("Auto Chooser", m_chooser);
    m_chooser.addOption("Two Ball Auto", new TwoBallAuto(m_driveSubsystem, m_shooterSubystem, m_towerSubsystem, m_intakeSubsystem));
    m_chooser.addOption("One Ball Auto", new OneBallAuto(m_driveSubsystem, m_shooterSubystem, m_towerSubsystem));

    SmartDashboard.putData("Auto Chooser", m_chooser);

    Limelight.initialize();
    Limelight.setDriverMode();

    // Configure the button bindings
    configureButtonBindings();

    // For simulation

	  // Set up the default commands for the various subsystems
    m_driveSubsystem.setDefaultCommand(new SwerveDrive(m_driveSubsystem, 
                                      () -> (m_driverController.getLeftX()) * DriveSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
                                      () -> -(m_driverController.getLeftY()) * DriveSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
                                      () -> -(m_driverController.getRightX()) * DriveSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND));
    
    m_intakeSubsystem.setDefaultCommand(new DriveIntake(m_intakeSubsystem,
                                        () -> (m_operatorController.getRightTriggerAxis()),  
                                        () -> (m_operatorController.getLeftTriggerAxis())));

    m_towerSubsystem.setDefaultCommand(new DriveTower(m_towerSubsystem,  
                                      () -> -m_operatorController.getLeftY()));

    m_climberSubsystem.setDefaultCommand(new ManualClimbByStick(m_climberSubsystem, 
                                        () -> m_operatorController.getRightY()));

    // Make the Climb Sequence commands available on SmartDash
    SmartDashboard.putData(new A0_CalibrateClimber(m_climberSubsystem));
    SmartDashboard.putData(new A1_PrepareToClimb(m_climberSubsystem /*, m_intakeSubsystem */));
    SmartDashboard.putData(new A2_LiftToBar(m_climberSubsystem));
    SmartDashboard.putData(new A3_ReachToNextBar(m_climberSubsystem));
    SmartDashboard.putData(new A4_HookToNextBar(m_climberSubsystem));
    
    // Arm Driving Commands
    SmartDashboard.putData(new ManualClimbByStick(m_climberSubsystem, () -> (-1.0) * m_operatorController.getRightY()));
    SmartDashboard.putData(new MagicClimbByStick(m_climberSubsystem, () -> (-1.0) * m_operatorController.getRightY()));
    SmartDashboard.putData(new MagicClimbByDash(m_climberSubsystem));

    SmartDashboard.putData(new InstantCommand(m_climberSubsystem::zeroSensors, m_climberSubsystem));

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    //Driver Controller
        
    new XBoxControllerButton(m_driverController, XBoxControllerEE.Button.kLeftBumper)
      .whileHeld(new IntakeOverride(m_intakeSubsystem, true));
    
    new XBoxControllerButton(m_driverController, XBoxControllerEE.Button.kRightBumper)
      .whileHeld(new IntakeOverride(m_intakeSubsystem, false));

    // Back button zeros the gyroscope
    new XBoxControllerButton(m_driverController, XBoxControllerEE.Button.kBack)
        .whenPressed(m_driveSubsystem::zeroGyroscope);

    //Operator controller    
    new XBoxControllerButton(m_operatorController, XBoxControllerEE.Button.kA)
      .whenHeld(new ShootLowerHub(m_shooterSubystem, m_towerSubsystem));

    new XBoxControllerButton(m_operatorController, XBoxControllerEE.Button.kB)
      .whenHeld(new ShootUpperHub(m_shooterSubystem, m_towerSubsystem));

    new XBoxControllerButton(m_operatorController, XBoxControllerEE.Button.kLeftBumper)
      .whileActiveContinuous(new InstantCommand(m_shooterSubystem::deployHood, m_shooterSubystem));
    
    new XBoxControllerButton(m_operatorController, XBoxControllerEE.Button.kRightBumper)
      .whileActiveContinuous(new InstantCommand(m_shooterSubystem::retractHood, m_shooterSubystem));

    new XBoxControllerButton(m_operatorController, XBoxControllerEE.Button.kA)
      .whenPressed(new InstantCommand(m_climberSubsystem::zeroSensors));
        
    new XBoxControllerDPad(m_operatorController, XBoxControllerEE.DPad.kDPadUp)
     .whileActiveContinuous(new InstantCommand(m_climberSubsystem::fixedClimberVertical));
    
    new XBoxControllerDPad(m_operatorController, XBoxControllerEE.DPad.kDPadDown)
      .whileActiveContinuous(new InstantCommand(m_climberSubsystem::fixedClimberAngled));

    new XBoxControllerDPad(m_operatorController, XBoxControllerEE.DPad.kDPadLeft)
      .whileActiveContinuous(new InstantCommand(m_climberSubsystem::extendingClimberAngled));
    
    new XBoxControllerDPad(m_operatorController, XBoxControllerEE.DPad.kDPadRight)
      .whileActiveContinuous(new InstantCommand(m_climberSubsystem::extendingClimberVertical));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_chooser.getSelected();
  }
}
