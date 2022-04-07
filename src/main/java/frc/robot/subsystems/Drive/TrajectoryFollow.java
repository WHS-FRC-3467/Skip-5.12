package frc.robot.subsystems.Drive;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TrajectoryFollow extends CommandBase {

    private String m_pathName;
    private PathPlannerTrajectory m_trajectory = null;
    DriveSubsystem m_drive;
    /**
     * Executes a trajectory that makes it remain still
     */
    public TrajectoryFollow() {
        m_pathName = "Stay Still";
    }

    public TrajectoryFollow(String pathName, DriveSubsystem drive) {
        m_pathName = pathName;
        m_drive = drive;
    }

    public TrajectoryFollow(Trajectory traj, DriveSubsystem drive) {
        m_trajectory = (PathPlannerTrajectory) traj;
        m_drive = drive;
    }

    @Override
    public void initialize() {
        System.out.println("Trajectory begun");

        if (m_trajectory == null) {
            try {
                m_trajectory = PathPlanner.loadPath(m_pathName, 1, 1); //2.9, 3
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
        m_drive.m_field.getObject("traj").setTrajectory(m_trajectory);

        ProfiledPIDController thetaController = new ProfiledPIDController(1, 0, 0,
                new TrapezoidProfile.Constraints(DriveSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
                        Math.pow(DriveSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, 2)));
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        new PPSwerveControllerCommand(m_trajectory,
                m_drive::getCurrentPose,
                m_drive.getKinematics(),
                new PIDController(10, 0, 0),
                new PIDController(10, 0, 0),
                thetaController,
                m_drive::actuateModulesAuto,
                m_drive)
                        .andThen(() -> m_drive.drive(new ChassisSpeeds(0.0, 0.0, 0.0)))
                        .schedule();    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }

}
