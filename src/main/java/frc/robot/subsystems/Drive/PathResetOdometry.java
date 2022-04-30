// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Drive;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class PathResetOdometry extends CommandBase {
    
    PathPlannerTrajectory trajectory = null;
    DriveSubsystem m_drive;
    public PathResetOdometry(String pathName, DriveSubsystem drive) {
        try {
            trajectory = PathPlanner.loadPath(pathName, 1, 1);
            System.out.println(pathName);
        } catch (Exception e) {
            e.printStackTrace();
        }        
        m_drive = drive;
    }

    @Override
    public void initialize() {
        Pose2d currentPose = m_drive.getCurrentPose();
        Pose2d initialPose = trajectory.getInitialPose();
        Rotation2d offsetRot = initialPose.getRotation().minus(currentPose.getRotation());

        Pose2d offsetPose = new Pose2d(
            initialPose.getX(),
            initialPose.getY(),
            offsetRot
        );

        m_drive.setGyroscope(offsetRot.getDegrees());
        
        m_drive.resetOdometry(offsetPose);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
}