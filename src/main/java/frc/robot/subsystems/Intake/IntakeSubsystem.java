// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanConstants;
import frc.robot.Constants.PHConstants;

public class IntakeSubsystem extends SubsystemBase {
  DoubleSolenoid m_intakePiston = new DoubleSolenoid(9, PneumaticsModuleType.REVPH, PHConstants.IntakeForwardSoleniod, PHConstants.IntakeReverseSoleniod);
  TalonFX m_intakeMotor = new TalonFX(CanConstants.IntakeMotor);
  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
public void driveIntake(double speed){
  m_intakeMotor.set(ControlMode.PercentOutput, speed);


}
public void intakeIn(){
  m_intakePiston.set(Value.kForward);
}
public void intakeOut(){
  m_intakePiston.set(Value.kForward);
}
public void toggleIntake(){
  m_intakePiston.toggle();
}

}


