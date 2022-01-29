// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climber;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanConstants;
import frc.robot.Constants.PHConstants;

public class ClimberSubsystem extends SubsystemBase {
 TalonFX m_climberMotorLeft = new TalonFX(CanConstants.ClimberLeft);
 TalonFX m_climberMotorRight = new TalonFX(CanConstants.ClimberRight);
  DoubleSolenoid m_climberPiston = new DoubleSolenoid(9, PneumaticsModuleType.REVPH, PHConstants.ClimberForwardSoleniod, PHConstants.ClimberReverseSoleniod);

 
  /** Creates a new IntakeSubsystem. */
  public ClimberSubsystem() {
    m_climberMotorLeft.setInverted(true);
    m_climberMotorLeft.follow(m_climberMotorRight);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


public void driveClimber(double speed) {
  m_climberMotorRight.set(ControlMode.PercentOutput, speed);

}
public void climberForward(){
m_climberPiston.set(Value.kForward);
}

public void climberReverse(){
m_climberPiston.set(Value.kReverse);
}

public void toggleClimber(){
m_climberPiston.toggle();  
}

}
