// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Climber;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClimberConstants;

public class HookToNextBarFast extends CommandBase {

  ClimberSubsystem m_climber;
  int m_climbPhase = 1;
  Timer m_timer = new Timer();
  boolean m_fast;

  public HookToNextBarFast(ClimberSubsystem climber, boolean fast) {
    m_climber = climber;
    m_fast = fast;
    addRequirements(m_climber);
  }

  @Override
  public void initialize() {
    m_climbPhase = 1;
    m_timer.reset();
  }

  @Override
  public void execute() {

    switch (m_climbPhase) {
    case 1:

      m_timer.start();
      // Use slower acceleration to start retracting AdjArms
      m_climber.setMotionAccel(ClimberConstants.kSlowMotionAccelFast);
      m_climber.adjustArmsMagically(ClimberConstants.kClimbingRetractedPostion);
      // Start moving FixArms to Angled position
      m_climber.fixedClimberAngled();
      // As AdjArms near the setpoint, start moving FixArms back to Vertical position
      if ((m_fast && m_climber.getATposition() < 160000) || m_climber.getATposition() < 45000) {
        m_climber.fixedClimberVertical();
      }
      // Once we reach setpoint, stop and reset accelration to normal speed
      if (m_climber.areArmsOnTarget()) {
        m_climbPhase = 3;
        m_timer.stop();
        m_timer.reset();
        m_climber.setMotionAccel(ClimberConstants.kMotionAccelerationFast);
      }
      break;

    case 2:
      //Wait for fixed arms to return to vertical
      m_timer.start();
      if (m_timer.hasElapsed(0.1)) {
          m_climbPhase = 3;
          m_timer.stop();
          m_timer.reset();
      }
      break;

    case 3:
        if(m_fast){
            m_climbPhase = 0;
            break;
        }
        // Drop down so the fixed hooks can get under the bar  
        m_climber.adjustArmsMagically(ClimberConstants.kFixedArmsFree);
        if (m_climber.areArmsOnTarget()) {
            m_climbPhase = 4;
        }
        break;

    case 4:
        // Pull up again to fully engage fixed hooks
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
