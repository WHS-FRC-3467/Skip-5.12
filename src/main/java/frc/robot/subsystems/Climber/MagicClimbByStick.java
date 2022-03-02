package frc.robot.subsystems.Climber;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClimberConstants;

public class MagicClimbByStick extends CommandBase {

    private ClimberSubsystem m_climber;
    private DoubleSupplier m_position;

    public MagicClimbByStick(ClimberSubsystem climber, DoubleSupplier position) {
        super();
        m_position = position;
        m_climber = climber;
        addRequirements(m_climber);
    }
    
    public void initialize() {
        //m_climber.zeroSensors();
    }

   @Override
    public void execute() {

        // Multiply modified stick by the total range of positions to get target position
        // Account for non-zero origin
        double setpoint = modifyAxis(m_position.getAsDouble());
        setpoint = (setpoint * (ClimberConstants.kFullExtendedPosition - ClimberConstants.kRestingRetractedPostion)) +
             ClimberConstants.kRestingRetractedPostion ;

        // Can't servo to negative value
        if (setpoint < 0.0) setpoint = 0.0;

        m_climber.adjustArmsMagically(setpoint);
    }

    @Override
    public boolean isFinished() {
        return false;
        //return m_climber.areArmsOnTarget();
    }

    public void end(boolean interrupted) {
        m_climber.adjustArmsManually(0.0);
    }

    private static double modifyAxis(double value) {
        
        // Apply Deadband
            double dband = 0.2;
            if (Math.abs(value) > dband) {
            if (value > 0.0) {
                return (value - dband) / (1.0 - dband);
            } else {
                return (value + dband) / (1.0 - dband);
            }
            } else {
            value = 0.0;
            }
        
        // Square the axis for better control range at low speeds
        value = Math.copySign(value * value, value);

        return value;
        }

}
