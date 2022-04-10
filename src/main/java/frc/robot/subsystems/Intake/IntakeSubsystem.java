// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanConstants;
import frc.robot.Constants.PHConstants;

public class IntakeSubsystem extends SubsystemBase {
  DoubleSolenoid m_intakePiston = new DoubleSolenoid(PneumaticsModuleType.REVPH, PHConstants.IntakeForwardSolenoid, PHConstants.IntakeReverseSolenoid);
  TalonFX m_intakeMotor = new TalonFX(CanConstants.IntakeMotor);

  public boolean intakeRetracted = true;
  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    m_intakeMotor.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 255);
    m_intakeMotor.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 255);
    m_intakeMotor.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 255);
    m_intakeMotor.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 255);
    m_intakeMotor.setStatusFramePeriod(StatusFrame.Status_17_Targets1, 255);
    m_intakeMotor.setStatusFramePeriod(StatusFrame.Status_9_MotProfBuffer, 255);
    m_intakeMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
public void driveIntake(double speed){
  m_intakeMotor.set(ControlMode.PercentOutput, speed);
}
public void intakeDeploy(){
  m_intakePiston.set(Value.kForward);
}
public void intakeRetract(){
  m_intakePiston.set(Value.kReverse);
}
public Value intakeValue(){
  return m_intakePiston.get();
}
public boolean intakePosition(){
  //true = retacted
  if(intakeValue() == Value.kReverse){
    return true;
  }
  else{
    return false;
  }
}

}




