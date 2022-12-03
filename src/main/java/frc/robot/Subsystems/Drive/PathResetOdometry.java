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
    //Initialize variables
    PathPlannerTrajectory trajectory = null;
    DriveSubsystem m_drive;

    /**
     * Constructor for PathResetOdometry
     * @param pathName the string key for the path
     * @param drive Drive Subsystem
     */
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
        //gets gets pose information from drive subsystem
        Pose2d currentPose = m_drive.getCurrentPose();
        //gets initial pose from path planner
        Pose2d initialPose = trajectory.getInitialPose();
        //gets offset rotation from pathplanner
        Rotation2d offsetRot = initialPose.getRotation().minus(currentPose.getRotation());

        //creates offset Pose2D from  initial pose and rotation from pathplanner
        Pose2d offsetPose = new Pose2d(
            initialPose.getX(),
            initialPose.getY(),
            offsetRot
        );
        //sets the gyroscope yaw to offset rotation of path
        m_drive.setGyroscope(offsetRot.getDegrees());
        //sets drivebase odometry to offset pose of path
        m_drive.resetOdometry(offsetPose);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        //ends immediately
        return true;
    }
    
}
