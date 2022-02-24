package frc.robot.subsystems.Shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanConstants;

public class FalconVelocity extends SubsystemBase
{ //implements ISpeedControl
     /* Hardware */
     TalonFX m_motorLeft = new TalonFX(CanConstants.ShooterLeft);
     TalonFX m_motorRight = new TalonFX(CanConstants.ShooterRight);
    /* Gains */
     double m_kP = 0.0;
     double m_kI = 0.0;
     double m_kD = 0.0;
     double m_kF = 0.0;
 
    /* Current Limiting */
    private SupplyCurrentLimitConfiguration talonCurrentLimit;
    private final boolean ENABLE_CURRENT_LIMIT = true;
    private final double CONTINUOUS_CURRENT_LIMIT = 25; //amps
    private final double TRIGGER_THRESHOLD_LIMIT = 35; //amps
    private final double TRIGGER_THRESHOLD_TIME = .2; //secs


     public FalconVelocity()
     {
         /* Factory Default all hardware to prevent unexpected behaviour */
         m_motorLeft.configFactoryDefault();
         m_motorRight.configFactoryDefault();
 
         /* Config sensor used for Primary PID [m_Velocity] */
         m_motorLeft.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 30);
 
         //m_motorLeft.configClosedloopRamp(0.3);
         //m_motorRight.configClosedloopRamp(0.3);        
         
        /* Instead of throttle ramping, use current limits to moderate the wheel acceleration */
         talonCurrentLimit = new SupplyCurrentLimitConfiguration(ENABLE_CURRENT_LIMIT,
                CONTINUOUS_CURRENT_LIMIT, TRIGGER_THRESHOLD_LIMIT, TRIGGER_THRESHOLD_TIME);
 
         m_motorLeft.configSupplyCurrentLimit(talonCurrentLimit);
         m_motorRight.configSupplyCurrentLimit(talonCurrentLimit);
 
         /* Set motors to Coast */
         m_motorLeft.setNeutralMode(NeutralMode.Coast);
         m_motorRight.setNeutralMode(NeutralMode.Coast);
         
         /*
          * Phase sensor accordingly. 
          * Positive Sensor Reading should match Green (blinm_king) Leds on Talon
          */
         m_motorLeft.setSensorPhase(false);
         m_motorRight.setSensorPhase(false);
         
         /* Config the peak and nominal outputs */
         m_motorLeft.configNominalOutputForward(0.0, 30);
         m_motorLeft.configNominalOutputReverse(0.0, 30);
         m_motorLeft.configPeakOutputForward(1.0, 30);
         m_motorLeft.configPeakOutputReverse(0.0, 30); // Don't go in reverse
 
         m_motorRight.configNominalOutputForward(0.0, 30);
         m_motorRight.configNominalOutputReverse(0.0, 30);
         m_motorRight.configPeakOutputForward(1.0, 30);
         m_motorRight.configPeakOutputReverse(0.0, 30); // Don't go in reverse
 
         /* Set up voltage compensation to maintain consistent velocities across a range of battery voltages */
         m_motorLeft.configVoltageCompSaturation(12.0);
         m_motorLeft.enableVoltageCompensation(true);
         m_motorRight.configVoltageCompSaturation(12.0);
         m_motorRight.enableVoltageCompensation(true);

         /* Invert motor2 and have it follow motor1 */
         m_motorRight.follow(m_motorLeft);
         m_motorRight.setInverted(false);
         m_motorLeft.setInverted(true);
     }
 
     public void updateGains(double kP, double kI, double kD, double kF)
     {
         // if PIDF coefficients have changed, write new values to controller
         if((m_kP != kP)) { m_motorLeft.config_kP(0, kP, 30); m_kP = kP; }
         if((m_kI != kI)) { m_motorLeft.config_kI(0, kI, 30); m_kI = kI; }
         if((m_kD != kD)) { m_motorLeft.config_kD(0, kD, 30); m_kD = kD; }
         if((m_kF != kF)) { m_motorLeft.config_kF(0, kF, 30); m_kF = kF; }
     }
 
     public int runVelocityPIDF(double targetVelocity)
     {
         // Convert RPM to raw units per 100ms
         double targetVelocity_UnitsPer100ms = targetVelocity * 2048 / 600;
                 
         // Set Velocity setpoint
         m_motorLeft.set(ControlMode.Velocity, targetVelocity_UnitsPer100ms);
 
         // Get current speed and convert back to RPM
         return (int)((double)m_motorLeft.getSelectedSensorVelocity() * 600 / 2048);
     }
 
     public double getError()
     {
         return m_motorLeft.getClosedLoopError();
     }
 
     public double getOutputPercent()
     {
         return (m_motorLeft.getMotorOutputPercent() * 100);
     }
 
     public void stop()
     {
        m_motorLeft.set(ControlMode.PercentOutput, 0.0);
     }

     public double getEncoderAverage(){
        return m_motorLeft.getSelectedSensorVelocity();
     }

     
  
}
