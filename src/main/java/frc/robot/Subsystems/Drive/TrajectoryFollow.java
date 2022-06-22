package frc.robot.Subsystems.Drive;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class TrajectoryFollow {
    //Initialize variables
    private String m_pathName;
    private PathPlannerTrajectory m_trajectory = null;
    DriveSubsystem m_drive;
    /**
     * Executes a trajectory that makes it remain still
     */
    public TrajectoryFollow() {
        m_pathName = "Stay Still";
    }

    /**
     * 
     * @param pathName String key for path
     * @param drive Drive subsytem
     */
    public TrajectoryFollow(String pathName, DriveSubsystem drive) {
        m_pathName = pathName;
        m_drive = drive;
    }

    /**
     * @param traj 
     * @param drive
     */
    public TrajectoryFollow(Trajectory traj, DriveSubsystem drive) {
        m_trajectory = (PathPlannerTrajectory) traj;
        m_drive = drive;
    }

    //returns a sequential command for PPSwerveControllerCommand that will begin when TrajectoryFollow(...).get() is called
    public SequentialCommandGroup get() {
        System.out.println("Trajectory begun");

        //If trajectory does not exist 
        if (m_trajectory == null) {
            try {
                m_trajectory = PathPlanner.loadPath(m_pathName, 9, 5); 
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
        //puts trajectory onto feild object
        m_drive.m_field.getObject("traj").setTrajectory(m_trajectory);

        // Controller for hollonomic rotation
        ProfiledPIDController thetaController = new ProfiledPIDController(1, 0, 0,
                new TrapezoidProfile.Constraints(DriveSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
                        Math.pow(DriveSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, 2)));
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        //the PPSwerveControllerCommand that moves the drive base in auto
        return new PPSwerveControllerCommand(m_trajectory,
                m_drive::getCurrentPose,
                m_drive.getKinematics(),
                new PIDController(10, 0, 0),
                new PIDController(10, 0, 0),
                thetaController,
                m_drive::actuateModulesAuto,
                m_drive)
                        .andThen(() -> m_drive.drive(new ChassisSpeeds(0.0, 0.0, 0.0)));
    }

}
