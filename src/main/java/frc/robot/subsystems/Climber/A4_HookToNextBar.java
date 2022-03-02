// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climber;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClimberConstants;

public class A4_HookToNextBar extends CommandBase {

  ClimberSubsystem m_climber;
  int m_climbPhase = 1;
  Timer m_timer = new Timer();

  public A4_HookToNextBar(ClimberSubsystem climber) {
    m_climber = climber;
    addRequirements(m_climber);
  }

  @Override
  public void initialize() {
    m_climbPhase = 1;
    m_timer.reset();
  }

  @Override
  public void execute() {

  //Extend AdjArms to BarClearance length
  switch (m_climbPhase) {
    case 1:

      //Start retracting AdjArms to FixedArmFree position and start moving FixArms to Angled position
      m_timer.start();
      m_climber.adjustArmsMagically(ClimberConstants.kClimbingRetractedPostion);
      if (m_timer.hasElapsed(1.0)) {
        m_climber.fixedClimberAngled();
        if (m_climber.areArmsOnTarget()) {
          m_climbPhase = 3;
          m_timer.stop();
          m_timer.reset();
          m_climber.fixedClimberVertical();
        }
      }
      break;

/*
    case 2:
      //Continue Retracting AdjArms to Minimum length and Move FixArms to Vertical position
      m_climber.adjustArmsMagically(ClimberConstants.kClimbingRetractedPostion);
      m_climber.fixedClimberVertical();

      if (m_climber.areArmsOnTarget()) {
        m_climbPhase = 3;
      }
      break;
*/
    case 3:
      // Drop down so the fixed hooks can get under the bar  
      m_climber.adjustArmsMagically(ClimberConstants.kFixedArmsFree);
      if (m_climber.areArmsOnTarget()) {
        m_climbPhase = 4;
      }
      break;

    case 4:
      // Pull up again  
      m_climber.adjustArmsMagically(ClimberConstants.kClimbingRetractedPostion);
      if (m_climber.areArmsOnTarget()) {
        m_climbPhase = 5;
      }
      break;

    case 5:
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
  }

  @Override
  public boolean isFinished() {
    return (m_climbPhase == 0);
  }
}
