
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanConstants;
import frc.robot.Constants.PHConstants;

public class IntakeSubsystem extends SubsystemBase {
  //Initializes solenoid and talon
  DoubleSolenoid m_intakePiston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, PHConstants.IntakeForwardSolenoid, PHConstants.IntakeReverseSolenoid);
  TalonFX m_intakeMotor = new TalonFX(CanConstants.IntakeMotor);
  TalonFX m_intakeMotor2 = new TalonFX(CanConstants.IntakeMotor2);
  
  public static Boolean m_running;
  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    m_intakeMotor.configFactoryDefault();
    m_intakeMotor.configFactoryDefault();
    m_intakeMotor.setNeutralMode(NeutralMode.Coast);
    m_intakeMotor2.setNeutralMode(NeutralMode.Coast);
    m_intakeMotor2.setInverted(true);
    m_intakeMotor.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 255);
    m_intakeMotor.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 255);
    m_intakeMotor.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 255);
    m_intakeMotor.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 255);
    m_intakeMotor.setStatusFramePeriod(StatusFrame.Status_17_Targets1, 255);
    m_intakeMotor.setStatusFramePeriod(StatusFrame.Status_9_MotProfBuffer, 255);
    m_intakeMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255);

    m_intakeMotor2.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 255);
    m_intakeMotor2.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 255);
    m_intakeMotor2.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 255);
    m_intakeMotor2.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 255);
    m_intakeMotor2.setStatusFramePeriod(StatusFrame.Status_17_Targets1, 255);
    m_intakeMotor2.setStatusFramePeriod(StatusFrame.Status_9_MotProfBuffer, 255);
    m_intakeMotor2.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255);


    m_intakeMotor.configOpenloopRamp(0.75);
    m_intakeMotor2.configOpenloopRamp(0.75);

    m_intakeMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 15, 20, 0.10));
    m_intakeMotor2.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 15, 20, 0.10));

    m_intakeMotor2.follow(m_intakeMotor);
    m_running = false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(m_intakeMotor.getMotorOutputPercent() > 0.1 || m_intakeMotor.getMotorOutputPercent() < -0.1){
      m_running = true;
    }
    else{
      m_running = false;
    }
    SmartDashboard.putBoolean("running", m_running);
    SmartDashboard.putNumber("Intake Current Draw", m_intakeMotor.getSupplyCurrent());
  }


  public static boolean getRunning(){
    return m_running;
  }
  /**
   * 
   * @param speed speed to set intake motor at 
   */
  public void driveIntake(double speed){
    m_intakeMotor.set(ControlMode.PercentOutput, speed * 1.0 );
 
  }
  //Runs intake at full speed
  public void fullRunIntake (){
    driveIntake(-1.0);
  }

  //Runs intake at full speed
  public void slowRunIntake (){
    driveIntake(-0.25);
  }
  //deploys intake
  public void deployIntake(){
    m_intakePiston.set(Value.kReverse);
  } 
  //retracts intake  
  public void retractIntake(){
    m_intakePiston.set(Value.kForward);
  }
  //gets the position of the intake
  public Value intakeValue(){
    return m_intakePiston.get();
  }
  //gets intake position
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




