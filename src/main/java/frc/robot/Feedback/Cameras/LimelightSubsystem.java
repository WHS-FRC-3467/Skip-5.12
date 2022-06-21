// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Feedback.Cameras;

import java.util.Map;

import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase {
  	/** Creates a new Limelight. */
	//   private static HttpCamera limelightFeed;
	public static NetworkTableInstance table = NetworkTableInstance.getDefault();
	public static HttpCamera limelightFeed;

	public LimelightSubsystem() {
	}

  	public static void initialize(){
		ShuffleboardTab dashboardTab = Shuffleboard.getTab("Driver Dash");
		limelightFeed = new HttpCamera("limelight", "http://limelight.local:5809/stream.mjpg");
		dashboardTab.add("LL", limelightFeed).withPosition(1,0).withSize(12, 6).withProperties(Map.of("Show Crosshair", true, "Show Controls", false));
		setDriverMode();
		setStreamMode(StreamMode.ePIPMain);
  	}
	@Override
	public void periodic() {
    	// This method will be called once per scheduler run
  	}
 	 public static enum CameraMode {
		eVision, eDriver
	}

	public static enum LightMode {
		ePipeline, eOff, eBlink, eOn
	}

	/**
	 * Stream modes for Limelight.
	 */
	public static enum StreamMode {
		eStandard, ePIPMain, ePIPSecondary
	}

	/**
	 * Sets LED mode of Limelight.
	 * 
	 * @param mode
	 *            Light mode for Limelight.
	 */
	public static void setLedMode(LightMode mode) {
		getValue("ledMode").setNumber(mode.ordinal());
	}

	/**
	 * Sets camera mode for Limelight.
	 * 
	 * @param mode
	 *            Camera mode for Limelight.
	 */
	public static void setCameraMode(CameraMode mode) {
		getValue("camMode").setNumber(mode.ordinal());
	}

	/**
	 * Sets pipeline number (0-9 value).
	 * 
	 * @param number
	 *            Pipeline number (0-9).
	 */
	public static void setPipeline(int number) {
		getValue("pipeline").setNumber(number);
	}

   	/**
	 * Sets streaming mode for Limelight.
	 * 
	 * @param mode
	 *            Stream mode for Limelight.
	 */
	public static void setStreamMode(StreamMode mode) {
		getValue("stream").setNumber(mode.ordinal());
	}


	/**
	 * Sets Limelight to "Driver" mode.
	 */
	public static void setDriverMode() {
        setCameraMode(CameraMode.eDriver);
        setPipeline(0);
        setLedMode(LightMode.ePipeline);
	}

	/**
	 * Sets Limelight to "Vision" mode with specified Pipeline.
	 */
	public static void setVisionMode(int pipelinenumber) {
        setCameraMode(CameraMode.eVision);
        setPipeline(pipelinenumber);
        setLedMode(LightMode.ePipeline);
	}

	/**
	 * Sets Limelight to "Vision" mode with Pipeline 1
	 */
	public static void setVisionMode() {
        setCameraMode(CameraMode.eVision);
        setPipeline(1);
        setLedMode(LightMode.ePipeline);
	}

    /**
	 * Turns Limelight LEDS off
	 */
	public static void turnOffLEDs() {
            setLedMode(LightMode.eOff);
	}

	private static NetworkTableEntry getValue(String key){
		return table.getTable("limelight").getEntry(key);
	}
}
