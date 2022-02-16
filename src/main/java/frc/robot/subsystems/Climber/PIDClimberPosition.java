package frc.robot.subsystems.Climber;
// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems.Climber;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.Constants.ClimberConstants;

// public class PIDClimberPosition extends CommandBase {
//   /** Creates a new PIDClimberPosition. */
//   ClimberSubsystem m_climber;
//   double m_position;
//   PIDController m_pidController = new PIDController(ClimberConstants.kP, ClimberConstants.kI, ClimberConstants.kD);

//   public PIDClimberPosition(ClimberSubsystem climber, double position) {
//     m_climber = climber;
//     m_position = position;
//     addRequirements(m_climber);
//     // Use addRequirements() here to declare subsystem dependencies.
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     m_pidController.setTolerance(0.5);
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     m_climber.driveClimber(m_pidController.calculate(m_climber.getEncoderAverage(), m_position));
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     m_climber.driveClimber(0.0);
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return m_pidController.atSetpoint();
//   }
// }
