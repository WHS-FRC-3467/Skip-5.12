package frc.robot.subsystems.Climber;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class MagicClimbByDash extends CommandBase {

    private ClimberSubsystem m_climber;

    public MagicClimbByDash(ClimberSubsystem climber) {

        super();
        m_climber = climber;
        addRequirements(m_climber);
    }

    public void initialize() {
        m_climber.zeroSensors();
    }

   @Override
    public void execute() {
        // This method will take setpoint value from SmartDash
        m_climber.adjustArmsMagically();
    }

    @Override
    public boolean isFinished() {
        //return false;
        return m_climber.areArmsOnTarget();
    }

    public void end(boolean interrupted) {
        m_climber.adjustArmsManually(0.0);
    }
    
}