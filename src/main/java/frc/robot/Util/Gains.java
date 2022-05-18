package frc.robot.Util;

public class Gains {
	public final double kP;
	public final double kI;
	public final double kD;
	public final double kF;
	public final int kIzone;
	public final double kPeakOutput;
	/**
	 * 
	 * @param _kP Proportional Gain
	 * @param _kI Integral Gain
	 * @param _kD Derivative Gain
	 * @param _kF Feed Forward Gain
	 * @param _kIzone Integral Zone
	 * @param _kPeakOutput Peak output
	 */
	public Gains(double _kP, double _kI, double _kD, double _kF, int _kIzone, double _kPeakOutput){
		kP = _kP;
		kI = _kI;
		kD = _kD;
		kF = _kF;
		kIzone = _kIzone;
		kPeakOutput = _kPeakOutput;
	}
}
