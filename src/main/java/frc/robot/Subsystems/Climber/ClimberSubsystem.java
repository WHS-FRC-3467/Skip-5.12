// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Climber;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.CanConstants;
import frc.robot.Constants.PHConstants;
//import frc.robot.sim.PhysicsSim;

public class ClimberSubsystem extends SubsystemBase {

  // Solenoid control
  DoubleSolenoid m_fixedClimberPiston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,
      PHConstants.FixedClimberVerticalSolenoid, PHConstants.FixedClimberAngledSolenoid);
  DoubleSolenoid m_extendingClimberPiston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,
      PHConstants.ExtendingClimberAngledSolenoid, PHConstants.ExtendingClimberVerticalSolenoid);

  // Climber Winch Motors
  TwinTalonFXMech m_talonMech = new TwinTalonFXMech(CanConstants.ClimberLeft, CanConstants.ClimberRight);

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {

    // Zero the encoders
    m_talonMech.zeroSensors();

  }

/*
  @Override
	public void simulationPeriodic() {
		PhysicsSim.getInstance().run();
	}
*/

  /*
   * Manual Extendable Arm Control
   */
  public void adjustArmsManually(double speed) {
    m_talonMech.drive(speed);
  }

  /*
   * Arm Calibration method
   */
  public boolean calibrateArm(boolean left) {
    return m_talonMech.calibrate(left);
  }

  /*
   * Motion Magic Extendable Arm Control
   */
  public void adjustArmsMagically(double position) {
    m_talonMech.runMotionMagic(position);
  }

  public void adjustArmsMagically() {
    m_talonMech.runMotionMagic();
  }

  public boolean areArmsOnTarget() {
    return m_talonMech.isMechOnTarget();
  }

  public double getATposition() {
    return m_talonMech.getATPosition();
  }

  public void setMotionAccel(double accel) {
    m_talonMech.setMotionAccel(accel);
  }

  public void zeroSensors() {
    m_talonMech.zeroSensors();
  }

  /*
   *  Pneumatic Arm Positioning
   */

  public void fixedClimberVertical() {
    m_fixedClimberPiston.set(Value.kForward);
  }

  public void fixedClimberAngled() {
    m_fixedClimberPiston.set(Value.kReverse);
  }

  public void extendingClimberAngled() {
    m_extendingClimberPiston.set(Value.kForward);
  }

  public void extendingClimberVertical() {
    m_extendingClimberPiston.set(Value.kReverse);
  }

}
