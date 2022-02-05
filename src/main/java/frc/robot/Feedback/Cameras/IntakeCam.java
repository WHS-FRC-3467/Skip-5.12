// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Feedback.Cameras;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeCam extends SubsystemBase {
  /** Creates a new IntakeCam. */
  public IntakeCam() {
    UsbCamera intakeCam = CameraServer.startAutomaticCapture("MS Lifecam Camera", 0);
    intakeCam.setResolution(320, 240);
    intakeCam.setFPS(15);
    intakeCam.setResolution(640, 480);
    intakeCam.setExposureAuto();
    intakeCam.setWhiteBalanceAuto();
    intakeCam.setConnectionStrategy(ConnectionStrategy.kKeepOpen);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
