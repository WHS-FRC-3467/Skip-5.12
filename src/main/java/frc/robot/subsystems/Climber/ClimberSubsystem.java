package frc.robot.subsystems.Climber;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj.DoubleSolenoid;
// import edu.wpi.first.wpilibj.PneumaticsModuleType;
// import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanConstants;
// import frc.robot.Constants.ClimberConstants;
// import frc.robot.Constants.PHConstants;

public class ClimberSubsystem extends SubsystemBase {
  TalonFX m_climberMotorLeft = new TalonFX(CanConstants.ClimberLeft);
  TalonFX m_climberMotorRight = new TalonFX(CanConstants.ClimberRight);
//   DoubleSolenoid m_fixedClimberPiston = new DoubleSolenoid(PneumaticsModuleType.REVPH, PHConstants.FixedClimberForwardSoleniod, PHConstants.FixedClimberReverseSoleniod);
//   DoubleSolenoid m_extenedingClimberPiston = new DoubleSolenoid(PneumaticsModuleType.REVPH, PHConstants.ExtendingClimberForwardSoleniod, PHConstants.ExtendingClimberReverseSoleniod);

//   PIDController m_pidController = new PIDController(ClimberConstants.kP, ClimberConstants.kI, ClimberConstants.kD);
 
  /** Creates a new IntakeSubsystem. */
  public ClimberSubsystem() {
    m_climberMotorLeft.setInverted(true);
    m_climberMotorLeft.follow(m_climberMotorRight);

    m_climberMotorRight.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    m_climberMotorLeft.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    
    m_climberMotorRight.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);
    m_climberMotorLeft.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void driveClimber(double speed) {
    m_climberMotorRight.set(ControlMode.PercentOutput, speed);
  }

//   public void fixedClimberForward(){
//   m_fixedClimberPiston.set(Value.kForward);
//   }

//   public void fixedClimberReverse(){
//   m_fixedClimberPiston.set(Value.kReverse);
//   }

//   public void extendingClimberForward(){
//     m_extenedingClimberPiston.set(Value.kForward);
//   }
  
//   public void extendingClimberReverse(){
//     m_extenedingClimberPiston.set(Value.kReverse);
//   }

//   public double getEncoderAverage(){
//     return m_climberMotorRight.getSelectedSensorPosition() + m_climberMotorLeft.getSelectedSensorPosition()/2;
//   }
  
//   public void climberUp(){
//     m_climberMotorRight.set(ControlMode.PercentOutput, m_pidController.calculate(getEncoderAverage(), ClimberConstants.extendedPosition));  
//   }

//   public void climberDown(){
//     m_climberMotorRight.set(ControlMode.PercentOutput, m_pidController.calculate(getEncoderAverage(), 0.0));  
//   }
}
