// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Climber;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;

public class X2_LiftToBar extends CommandBase {

  ClimberSubsystem m_climber;
  int m_climbPhase = 1;
  Timer m_timer = new Timer();

  public X2_LiftToBar(ClimberSubsystem climber) {
    m_climber = climber;
    addRequirements(m_climber);
  }

  @Override
  public void initialize() {
    m_climbPhase = 1;
    m_timer.reset();
    m_climber.setMotionAccel(Constants.ClimberConstants.kMotionAcceleration);
  }

  @Override
  public void execute() {
    
    switch (m_climbPhase) {

      case 1:
        // Retract Arms to Minimum Length
        m_climber.adjustArmsMagically(ClimberConstants.kClimbingRetractedPostion);
        if (m_climber.areArmsOnTarget()) {
          m_climbPhase = 2;
        }
        break;

      case 2:
        // Extend arms above the bar so the fixed hooks settle onto bar  
        m_climber.adjustArmsMagically(ClimberConstants.kExtendedAboveBar);
        if (m_climber.areArmsOnTarget()) {
          m_climbPhase = 0;  // Finished
        }
      break;

      default:
        SmartDashboard.putString("status", "Phase ???");
        break;
    } 
  }

  @Override
  public void end(boolean interrupted) {
    m_climber.adjustArmsManually(0.0);
    m_climber.setMotionAccel(Constants.ClimberConstants.kMotionAcceleration);
  }

  @Override
  public boolean isFinished() {
    return (m_climbPhase == 0);
  }
}
