// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drive;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class PathResetOdometry extends CommandBase {
    
    PathPlannerTrajectory trajectory = null;
    private double offset_;
    DriveSubsystem m_drive;
    public PathResetOdometry(String pathName, DriveSubsystem drive) {
        try {
            trajectory = PathPlanner.loadPath(pathName, 8, 5);
        } catch (Exception e) {
            e.printStackTrace();
        }
        offset_ = 0;
        m_drive = drive;
    }

    public PathResetOdometry(String pathName, double offset) {
        try {
            trajectory = PathPlanner.loadPath(pathName, 8, 5);
        } catch (Exception e) {
            e.printStackTrace();
        }
        //double offset_ = offset;
    }

    @Override
    public void initialize() {
        Pose2d initialPose = trajectory.getInitialPose();
        Pose2d offsetPose = new Pose2d(
            initialPose.getX(),
            initialPose.getY(),
            new Rotation2d(initialPose.getRotation().getDegrees() + offset_)
        );
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