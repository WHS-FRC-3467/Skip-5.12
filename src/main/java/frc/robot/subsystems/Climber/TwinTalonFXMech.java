// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Climber;

//import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import frc.robot.Robot;
import frc.robot.Constants.ClimberConstants;
//import frc.robot.sim.PhysicsSim;

/**
 *  This class manages Motion Magic control in the two separate winch Talon 500s
 */

public class TwinTalonFXMech {

    /* Enable SmartDash Output? */
	boolean m_debugging = false;	
	int m_debug_counter = 0;	
		
    /** Hardware */
	WPI_TalonFX m_leftFollower;
	WPI_TalonFX m_rightMaster;
	
	/** Invert Directions for Left and Right */
	TalonFXInvertType m_leftInvert = TalonFXInvertType.CounterClockwise; //Same as invert = "false"
	TalonFXInvertType m_rightInvert = TalonFXInvertType.Clockwise; //Same as invert = "true"

	/** Config Objects for motor controllers */
	TalonFXConfiguration m_leftConfig = new TalonFXConfiguration();
	TalonFXConfiguration m_rightConfig = new TalonFXConfiguration();
	
    /** Default setpoint */
    private double m_setpoint = 0.0;

    public TwinTalonFXMech (int leftTalonID, int rightTalonID) {

		/* Instantiate Hardware */
        m_leftFollower = new WPI_TalonFX(leftTalonID, "rio");
        m_rightMaster = new WPI_TalonFX(rightTalonID, "rio");
        
		/* Disable all motor controllers */
		m_rightMaster.set(TalonFXControlMode.PercentOutput, 0);
		m_leftFollower.set(TalonFXControlMode.PercentOutput, 0);
		
		/* Set Neutral Mode */
		m_leftFollower.setNeutralMode(NeutralMode.Brake);
		m_rightMaster.setNeutralMode(NeutralMode.Brake);
		
		/** Feedback Sensor Configuration */
		
		/* Configure each Talon's selected sensor as local Integrated Sensor */
		m_leftConfig.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();	// Local Feedback Source
		m_leftConfig.primaryPID.selectedFeedbackCoefficient = 1.0;

		m_rightConfig.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();
		m_rightConfig.primaryPID.selectedFeedbackCoefficient = 1.0;

		/* Configure neutral deadband */
		m_rightConfig.neutralDeadband = ClimberConstants.kNeutralDeadband;
		m_leftConfig.neutralDeadband = ClimberConstants.kNeutralDeadband;
		
		/* Motion Magic Configurations */
		m_rightConfig.motionAcceleration = ClimberConstants.kMotionAcceleration;
		m_rightConfig.motionCruiseVelocity = ClimberConstants.kMotionCruiseVelocity;
		m_leftConfig.motionAcceleration = ClimberConstants.kMotionAcceleration;
		m_leftConfig.motionCruiseVelocity = ClimberConstants.kMotionCruiseVelocity;

		/**
		 * Max out the peak output (for all modes).  
		 * However you can limit the output of a given PID object with configClosedLoopPeakOutput().
		 */
		m_leftConfig.peakOutputForward = +1.0;
		m_leftConfig.peakOutputReverse = -1.0;
		m_rightConfig.peakOutputForward = +1.0;
		m_rightConfig.peakOutputReverse = -1.0;

		/* FPID Gains for distance servo */
		m_rightConfig.slot0.kP = ClimberConstants.kGains_Distance.kP;
		m_rightConfig.slot0.kI = ClimberConstants.kGains_Distance.kI;
		m_rightConfig.slot0.kD = ClimberConstants.kGains_Distance.kD;
		m_rightConfig.slot0.kF = ClimberConstants.kGains_Distance.kF;
		m_rightConfig.slot0.integralZone = ClimberConstants.kGains_Distance.kIzone;
		m_rightConfig.slot0.closedLoopPeakOutput = ClimberConstants.kGains_Distance.kPeakOutput;
		m_rightConfig.slot0.allowableClosedloopError = 0;

		m_leftConfig.slot0.kP = ClimberConstants.kGains_Distance.kP;
		m_leftConfig.slot0.kI = ClimberConstants.kGains_Distance.kI;
		m_leftConfig.slot0.kD = ClimberConstants.kGains_Distance.kD;
		m_leftConfig.slot0.kF = ClimberConstants.kGains_Distance.kF;
		m_leftConfig.slot0.integralZone = ClimberConstants.kGains_Distance.kIzone;
		m_leftConfig.slot0.closedLoopPeakOutput = ClimberConstants.kGains_Distance.kPeakOutput;
		m_leftConfig.slot0.allowableClosedloopError = 0;

		/**
		 * 1ms per loop.  PID loop can be slowed down if need be.
		 * For example,
		 * - if sensor updates are too slow
		 * - sensor deltas are very small per update, so derivative error never gets large enough to be useful.
		 * - sensor movement is very slow causing the derivative error to be near zero.
		 */
		int closedLoopTimeMs = 1;
		m_rightConfig.slot0.closedLoopPeriod = closedLoopTimeMs;
		m_leftConfig.slot0.closedLoopPeriod = closedLoopTimeMs;

		m_leftFollower.configAllSettings(m_leftConfig);
		m_rightMaster.configAllSettings(m_rightConfig);
		
		/* Configure output and sensor direction */
		m_leftFollower.setInverted(m_leftInvert);
		m_rightMaster.setInverted(m_rightInvert);
		/*
		 * Talon FX does not need sensor phase set for its integrated sensor
		 * This is because it will always be correct if the selected feedback device is integrated sensor (default value)
		 * and the user calls getSelectedSensor* to get the sensor's position/velocity.
		 * 
		 * https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#sensor-phase
		 */
        // m_leftFollower.setSensorPhase(true);
        // m_rightMaster.setSensorPhase(true);
		
		/* Set status frame periods to ensure we don't have stale data */
		m_rightMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 10);
		m_rightMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 10);
		m_leftFollower.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, ClimberConstants.kTimeoutMs);
		m_leftFollower.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 10);

        /* Configure Smoothing */
        m_rightMaster.configMotionSCurveStrength(ClimberConstants.kCurveSmoothing);
        m_leftFollower.configMotionSCurveStrength(ClimberConstants.kCurveSmoothing);

		/* Determine which slot affects which PID */
		m_rightMaster.selectProfileSlot(0, 0);
		m_leftFollower.selectProfileSlot(0, 0);

        /* Initialize sensors */
		zeroSensors();

		/* Set up the Talons in Simulation */
	/*
		if (Robot.isSimulation()) {
			// simulationInit
			PhysicsSim.getInstance().addTalonFX(m_leftFollower, 0.5, 21777, false);
			PhysicsSim.getInstance().addTalonFX(m_rightMaster, 0.5, 21777, false);
		}
	*/
	}
	
	/*
     * Manual control
     */
	public void drive(double speed) {
		m_rightMaster.set(ControlMode.PercentOutput, speed);
		m_leftFollower.set(ControlMode.PercentOutput, speed);

		reportMotionToDashboard();
	}


	public boolean calibrate(boolean left) {

		boolean isFinished = false;
		double current = 0.0;
		TalonFX talon = (left ? m_leftFollower : m_rightMaster);

		current = talon.getStatorCurrent();
		// SmartDashboard.putNumber("Calib Curr " + (left ? "L: " : "R: "), current);
		if (current < ClimberConstants.kCalibCurrentLimit) {
			talon.set(ControlMode.PercentOutput, -0.20);
			isFinished = false;
		} else {
			talon.set(ControlMode.PercentOutput, 0.0);
			isFinished = true;
		}
		return isFinished;
	}

	public void runMotionMagic() {
		
		// Run Motion Magic on each Talon using current value of m_setPoint as target

		m_rightMaster.set(TalonFXControlMode.MotionMagic, m_setpoint);
		m_leftFollower.set(TalonFXControlMode.MotionMagic, m_setpoint);

		reportMotionToDashboard();
	}
	
	public void runMotionMagic(double setpoint) {
	
		// Run Motion Magic on each Talon using passed-in setPoint as target

    	m_setpoint = setpoint;

		m_rightMaster.set(TalonFXControlMode.MotionMagic, setpoint);
		m_leftFollower.set(TalonFXControlMode.MotionMagic, setpoint);

		reportMotionToDashboard();
	}
	
	// Loop counter for determining stability around target setpoint
	int m_withinThresholdLoops = 0;
	
	public boolean isMechOnTarget() {
	
		final double kErrThreshold = ClimberConstants.kTolerance; // how many sensor units until it's close-enough?
		final int kLoopsToSettle = 10; // # of loops for which the sensor must be close-enough
	
		// Get current target and determine how far we are from it (error)
		double target = m_rightMaster.getClosedLoopTarget();
		double rightError = (target - m_rightMaster.getActiveTrajectoryPosition()); // Use this for Motion Magic
		target = m_leftFollower.getClosedLoopTarget();
		double leftError = (target - m_leftFollower.getActiveTrajectoryPosition()); // Use this for Motion Magic
		double avgError = (rightError + leftError)/2.0;

		/* Check if closed loop error is within the threshld */
		if (Math.abs(avgError) <= kErrThreshold) {
			++m_withinThresholdLoops;
		} else {
			m_withinThresholdLoops = 0;
		}
		return (m_withinThresholdLoops > kLoopsToSettle);
	}

	public double getATPosition() {
		return m_rightMaster.getActiveTrajectoryPosition();
	}

	public void setMotionAccel(double accel) {
		m_rightMaster.configMotionAcceleration(accel);
		m_leftFollower.configMotionAcceleration(accel);
	}

	public void reportMotionToDashboard() {

		if (m_debugging && ++m_debug_counter > 10) {
			// SmartDashboard.putString("Arms ControlMode", getTalonControlMode());
	    	SmartDashboard.putNumber("Right Arm Position", m_rightMaster.getSelectedSensorPosition(0));
	    	SmartDashboard.putNumber("Left Arm Position", m_leftFollower.getSelectedSensorPosition(0));
			// SmartDashboard.putNumber("Arms MotorOutputPercent", m_rightMaster.getMotorOutputPercent());
			// SmartDashboard.putNumber("Right Arm Draw", m_rightMaster.getStatorCurrent());
			// SmartDashboard.putNumber("Left Arm Draw", m_leftFollower.getStatorCurrent());
	    	
			if (m_rightMaster.getControlMode() == ControlMode.MotionMagic) {
				// SmartDashboard.putNumber("Arms Traj. Position", m_rightMaster.getActiveTrajectoryPosition());
				// SmartDashboard.putNumber("Arms ClosedLoopTarget", m_rightMaster.getClosedLoopTarget(0));
				// SmartDashboard.putNumber("Arms ClosedLoopError", m_rightMaster.getClosedLoopError(0));
			}

			// SmartDashboard.putNumber("P", getP());
			// SmartDashboard.putNumber("I", getI());
			// SmartDashboard.putNumber("D", getD());
			// SmartDashboard.putNumber("F", getF());
			// SmartDashboard.putNumber("Setpoint", getSetpoint());
			// SmartDashboard.putNumber("Accel", getMMAccel());
			// SmartDashboard.putNumber("Cruise", getMMCruise());
	  
			m_debug_counter = 0;
		}
	}
	
	/**
	 * @return The current TalonSRX control mode
	 */
	public String getTalonControlMode() {
		
		ControlMode tcm = m_rightMaster.getControlMode();
		
		if (tcm == ControlMode.PercentOutput) {
			return "PercentOutput";
		}
		else if (tcm == ControlMode.MotionMagic) {
			return "MotionMagic";
		}
		else
			return "Other";
	}

	/** Zero integrated encoders on Talons */
	void zeroSensors() {
		m_leftFollower.getSensorCollection().setIntegratedSensorPosition(0, ClimberConstants.kTimeoutMs);
		m_rightMaster.getSensorCollection().setIntegratedSensorPosition(0, ClimberConstants.kTimeoutMs);
		//System.out.println("[Integrated Sensors] All sensors are zeroed.\n");
		reportMotionToDashboard();
	}
	
    /** Configure Curve Smoothing */
    void configureSmoothing(int smooth) {
        m_rightMaster.configMotionSCurveStrength(smooth);
        m_leftFollower.configMotionSCurveStrength(smooth);
    }

    /** Return Talon Objects */
    TalonFX getLeftTalon() {
        return (TalonFX)m_leftFollower;
    }

    TalonFX getRightTalon() {
        return (TalonFX)m_rightMaster;
    }

	//private void setSetpoint(double sp)  { m_setpoint = sp; }
	// private double getSetpoint()  { return m_setpoint; }
    
	// //private void setP(double p)  {m_rightMaster.selectProfileSlot(0, 0); m_rightMaster.config_kP(0, p); m_leftFollower.selectProfileSlot(0, 0); m_leftFollower.config_kP(0, p);}
	// private double getP()  { return m_rightMaster.configGetParameter(ParamEnum.eProfileParamSlot_P, 0); }

	// //private void setI(double i)  {m_rightMaster.selectProfileSlot(0, 0); m_rightMaster.config_kI(0, i);  m_leftFollower.selectProfileSlot(0, 0); m_leftFollower.config_kI(0, i);}
	// private double getI()  { return m_rightMaster.configGetParameter(ParamEnum.eProfileParamSlot_I, 0); }

	// //private void setD(double d)  {m_rightMaster.selectProfileSlot(0, 0); m_rightMaster.config_kD(0, d);  m_leftFollower.selectProfileSlot(0, 0); m_leftFollower.config_kD(0, d);}
	// private double getD()  { return m_rightMaster.configGetParameter(ParamEnum.eProfileParamSlot_D, 0); }

	// //private void setF(double f)  {m_rightMaster.selectProfileSlot(0, 0); m_rightMaster.config_kF(0, f, 0);  m_leftFollower.selectProfileSlot(0, 0); m_leftFollower.config_kF(0, f);}
	// private double getF()  { return m_rightMaster.configGetParameter(ParamEnum.eProfileParamSlot_F, 0, 0); }

	// //private void setMMAccel(double acc)  {m_rightMaster.selectProfileSlot(0, 0); m_rightMaster.configMotionAcceleration(acc);  m_leftFollower.selectProfileSlot(0, 0); m_leftFollower.configMotionAcceleration(acc);}
	// private double getMMAccel()  { return m_rightMaster.configGetParameter(ParamEnum.eMotMag_Accel, 0, 0); }

	// //private void setMMCruise(double cru)  {m_rightMaster.selectProfileSlot(0, 0); m_rightMaster.configMotionCruiseVelocity(cru);  m_leftFollower.selectProfileSlot(0, 0); m_leftFollower.configMotionCruiseVelocity(cru);}
	// private double getMMCruise()  { return m_rightMaster.configGetParameter(ParamEnum.eMotMag_VelCruise, 0, 0); }

/*
	@Override
	public void initSendable(SendableBuilder builder) {
	  builder.setSmartDashboardType("Robot Preferences");
	  builder.addDoubleProperty("P", this::getP, this::setP);
	  builder.addDoubleProperty("I", this::getI, this::setI);
	  builder.addDoubleProperty("D", this::getD, this::setD);
	  builder.addDoubleProperty("F", this::getF, this::setF);
	  builder.addDoubleProperty("Setpoint", this::getSetpoint, this::setSetpoint);
	  builder.addDoubleProperty("Accel", this::getMMAccel, this::setMMAccel);
	  builder.addDoubleProperty("Cruise", this::getMMCruise, this::setMMCruise);
	}
*/
}
