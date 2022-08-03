package frc.robot.Subsystems.Shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
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
import frc.robot.Constants.ShooterConstants;
import frc.robot.Util.Gains;
import frc.robot.Util.TunableNumber;

public class ShooterSubsystem extends SubsystemBase
{ 
    /* Hardware */
    TalonFX m_motorLeft = new TalonFX(CanConstants.ShooterLeft);
    TalonFX m_motorRight = new TalonFX(CanConstants.ShooterRight);
    DoubleSolenoid m_hood = new DoubleSolenoid(PneumaticsModuleType.REVPH, PHConstants.HoodForwardSolenoid, PHConstants.HoodReverseSolenoid);

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

    //Gains for shooter tuning
    private static TunableNumber kPTest = new TunableNumber("Shooter kP");
    private static TunableNumber kITest = new TunableNumber("Shooter kI");
    private static TunableNumber kDTest = new TunableNumber("Shooter kD");
    private static TunableNumber kFTest = new TunableNumber("Shooter kF");
    private static TunableNumber kIzoneTest = new TunableNumber("Shooter Izone");

    private static TunableNumber kShooterSetpoint = new TunableNumber("Shooter Setpoint");

    public double setpoint;

    private Gains testGains;

     public ShooterSubsystem()
     {
         /* Factory Default all hardware to prevent unexpected behaviour */
         m_motorLeft.configFactoryDefault();
         m_motorRight.configFactoryDefault();
 
         /* Config sensor used for Primary PID [m_Velocity] */
         m_motorLeft.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 30);
 
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
        
         //slows unneeded status fames 
         m_motorLeft.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 255);
         m_motorLeft.setStatusFramePeriod(StatusFrame.Status_10_Targets, 255);
         m_motorLeft.setStatusFramePeriod(StatusFrame.Status_9_MotProfBuffer, 255);

         m_motorRight.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 255);
         m_motorRight.setStatusFramePeriod(StatusFrame.Status_10_Targets, 255);
         m_motorRight.setStatusFramePeriod(StatusFrame.Status_9_MotProfBuffer, 255);

         //sets default numbers for dashboard
         kPTest.setDefault(ShooterConstants.kShooterGains.kP);
         kITest.setDefault(ShooterConstants.kShooterGains.kI);
         kDTest.setDefault(ShooterConstants.kShooterGains.kD);
         kFTest.setDefault(ShooterConstants.kShooterGains.kF);
         kIzoneTest.setDefault(ShooterConstants.kShooterGains.kIzone);
         
         kShooterSetpoint.setDefault(0.0);

     }


    @Override
    public void periodic() {
        //Puts numbers to smart dashboard
        SmartDashboard.putNumber("Left Motor Current", getLeftMotorCurrent());
        SmartDashboard.putNumber("Right Motor Current", getRightMotorCurrent());

        SmartDashboard.putNumber("Current Velocity", getShooterVelocity());
    }
     /**
      * @param gains Gains for shooter
      */
     public void updateGains(Gains gains)
     {
        m_motorLeft.config_kP(0, gains.kP, 30); 
        m_motorLeft.config_kI(0, gains.kI, 30);
        m_motorLeft.config_kD(0, gains.kD, 30);
        m_motorLeft.config_kF(0,  gains.kF, 30);
        m_motorLeft.config_IntegralZone(0, gains.kIzone, 30);
     }

 
    /** 
    * @param targetVelocity the velocity in RPM of the shooter
    * @param gains Gains for shooter
    * @param hoodPosition The hood position in kFoward (deployed) or kReverse (retracted)
     */
    public void shoot(double targetVelocity, Value hoodPosition)
    {
        m_hood.set(hoodPosition);

        updateGains(ShooterConstants.kShooterGains);
        setpoint = targetVelocity;

        // Convert RPM to raw units per 100ms
        double targetVelocity_UnitsPer100ms = targetVelocity * 2048 / 600;
                
        // Set Velocity setpoint
        m_motorLeft.set(ControlMode.Velocity, targetVelocity_UnitsPer100ms);

        // SmartDashboard.putNumber("Current Velocity", getShooterVelocity());
    }
 

    /**
     * @return the error of the shooter from target RPM
     */
    public double getError()
    {
        return m_motorLeft.getClosedLoopError();
    }
    /**
     * @return the percent output of current from the left shooter motor
     */
    public double getOutputPercent()
    {
        return (m_motorLeft.getMotorOutputPercent() * 100);
    }
    
    public void stop()
    {
        m_motorLeft.set(ControlMode.PercentOutput, 0.0);
    }
    /**
     * @return The average raw encoder value of the left shooter motor
     */
    public double getEncoderAverage(){
        return m_motorLeft.getSelectedSensorVelocity();
    }

    public void runPercentOutput(double percent){
        m_motorLeft.set(ControlMode.PercentOutput, percent);
    }

    /**
     * @return the velocity of the shooter in RPM
     */
    public double getShooterVelocity(){
        return (int)((double)m_motorLeft.getSelectedSensorVelocity() * 600 / 2048);
    }

    public double getLeftMotorVoltage(){
        return m_motorLeft.getMotorOutputVoltage();
    }
    public double getRightMotorVoltage(){
        return m_motorRight.getMotorOutputVoltage();
    }

    public double getLeftMotorCurrent(){
        return m_motorLeft.getStatorCurrent();
    }
    public double getRightMotorCurrent(){
        return m_motorRight.getStatorCurrent();
    }
    public void retractHood(){
        m_hood.set(Value.kReverse);
    }

    public void deployHood(){
        m_hood.set(Value.kForward);
    }

    /**
     * @return true if the error of the shooter is within the tolerance
     */
    public boolean isWheelAtSpeed()
    {
        System.out.println(setpoint);
        System.out.println(getShooterVelocity());

        return Math.abs(setpoint - getShooterVelocity()) < ShooterConstants.kShooterTolerance;
    }

    public void stopShooter()
    {
        m_motorLeft.set(ControlMode.PercentOutput, 0.0);
    }

    
    
    public void testShoot(){
        testGains = new Gains(kPTest.get(), kITest.get(), kDTest.get(), kFTest.get(), kIzoneTest.get(), 1.0);

        updateGains(testGains);

        setpoint = kShooterSetpoint.get();

        // Update the target velocity and get back the current velocity
        double targetVelocity_UnitsPer100ms = kShooterSetpoint.get() * 2048 / 600;
                
        // Set Velocity setpoint
        m_motorLeft.set(ControlMode.Velocity, targetVelocity_UnitsPer100ms);
    }

}
