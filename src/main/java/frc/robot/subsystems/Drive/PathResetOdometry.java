// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drive;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class PathResetOdometry extends CommandBase {
    
    PathPlannerTrajectory trajectory = null;
    DriveSubsystem m_drive;
    public PathResetOdometry(String pathName, DriveSubsystem drive) {
        try {
            trajectory = PathPlanner.loadPath(pathName, 8, 5);
        } catch (Exception e) {
            e.printStackTrace();
        }
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
        PathPlannerState initialState = trajectory.getInitialState();
        Pose2d startingPose = new Pose2d(initialState.poseMeters.getTranslation(), initialState.holonomicRotation);
        m_drive.resetOdometry(startingPose);      
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