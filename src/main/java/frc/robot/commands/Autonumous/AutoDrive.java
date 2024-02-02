package frc.robot.commands.Autonumous;

import java.util.List;

import javax.tools.StandardJavaFileManager.PathFactory;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import edu.wpi.first.math.trajectory.Trajectory;
import frc.robot.subsystems.SwerveSubsystem;
public class AutoDrive extends SequentialCommandGroup  {
    SwerveSubsystem subsystem;
    public AutoDrive(SwerveSubsystem subsystem){
        this.subsystem=subsystem;
        PathPlannerPath path;
        path=PathPlannerPath.fromPathFile("Test Path");
        ChassisSpeeds chassisSpeeds;
        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
              0, 0, 0,
              subsystem.getRotation2d());
        PathPlannerTrajectory traj=new PathPlannerTrajectory(path,chassisSpeeds,
        this.subsystem.getRotation2d());
        
        addCommands(
            );
    }
}
