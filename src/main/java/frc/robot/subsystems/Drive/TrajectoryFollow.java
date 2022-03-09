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

    /**
     * Executes a trajectory that makes it remain still
     */
    public TrajectoryFollow() {
        m_pathName = "Stay Still";
    }

    public TrajectoryFollow(String pathName) {
        m_pathName = pathName;
    }

    public TrajectoryFollow(Trajectory traj) {
        m_trajectory = (PathPlannerTrajectory) traj;
    }

    @Override
    public void initialize() {
        System.out.println("Trajectory begun");


        if (m_trajectory == null) {
            try {
                m_trajectory = PathPlanner.loadPath(m_pathName, 8, 5); //2.9, 3
            } catch (Exception e) {
                e.printStackTrace();
            }
        }

        ProfiledPIDController thetaController = new ProfiledPIDController(5, 0, 0,
                new TrapezoidProfile.Constraints(DriveSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
                        Math.pow(DriveSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, 2)));
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        new PPSwerveControllerCommand(m_trajectory,
                DriveSubsystem.getInstance()::getCurrentPose,
                DriveSubsystem.getInstance().getKinematics(),
                new PIDController(6, 0, 0),
                new PIDController(6, 0, 0),
                thetaController,
                DriveSubsystem.getInstance()::actuateModulesAuto,
                DriveSubsystem.getInstance())
                        .andThen(() -> DriveSubsystem.getInstance().drive(new ChassisSpeeds(0.0, 0.0, 0.0)))
                        .schedule();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }

}
