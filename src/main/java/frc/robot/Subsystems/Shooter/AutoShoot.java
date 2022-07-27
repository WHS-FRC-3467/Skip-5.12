// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.TowerConstants;
import frc.robot.Subsystems.Tower.TowerSubsystem;
import frc.robot.Util.Gains;

public class AutoShoot extends CommandBase {
  ShooterSubsystem m_shooter;
  TowerSubsystem m_tower;
  double m_velocity, m_time, m_startTime;
  Gains m_gains;
  Value m_hoodPosition;
  Boolean m_end;
  double count;
  /**
   * @param shooter Shooter Subsystem
   * @param tower Tower Subsystem
   * @param velocity The velocity of the shooter in RPM
   * @param gains The gains for the shooter
   * @param hoodPosition The hood position in kFoward (deployed) or kReverse (retracted)
   */
  public AutoShoot(ShooterSubsystem shooter, TowerSubsystem tower, double velocity, Gains gains, Value hoodPosition) {
    m_shooter = shooter;
    m_tower = tower;
    m_gains = gains;
    m_hoodPosition = hoodPosition;
    m_velocity = velocity;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooter, m_tower);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //starts timer
    m_startTime = Timer.getFPGATimestamp();
    m_end = false;
    count = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //updates time
    m_time = Timer.getFPGATimestamp() - m_startTime;
    //runs shooter at velocity, updates gains in subsystem, sets hood to position
    m_shooter.shoot(m_velocity, m_hoodPosition);  
    
    //if wheel is up to speed or has been runing for more than 1.5 seconds then it will run tower if not it stops tower
    if(m_shooter.isWheelAtSpeed() || m_time >1.5 ){
      m_tower.driveWholeTower(TowerConstants.STANDARD_TOWER_SPEED);
    }
    else{
      m_tower.driveWholeTower(0.0);
    }

    System.out.println(m_end);
    System.out.println(m_tower.ballCount());

    //if ballcount is 0 then command will end if not it won't
    if(m_tower.ballCount() > 0){
      m_end = false;
      
    }
    else if(m_tower.ballCount() == 0){
      m_end = true;
    }

    System.out.println("Running Shooting");
    //Increases count
    count++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //Stops shooter and tower
    m_shooter.stopShooter();
    m_tower.driveWholeTower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return false;
  }
}
