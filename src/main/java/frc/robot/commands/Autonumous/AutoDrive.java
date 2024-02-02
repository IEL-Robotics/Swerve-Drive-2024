package frc.robot.commands.Autonumous;

import java.util.List;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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
    TrajectoryConfig config;
    Trajectory trajectory;
    ProfiledPIDController thetaController;
    public AutoDrive(SwerveSubsystem subsystem){
        this.subsystem=subsystem;
        this.config =
            new TrajectoryConfig(
                    Constants.SwerveConstants.AutoConstants.kMaxSpeedMetersPerSecond,
                    Constants.SwerveConstants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.SwerveConstants.DriveConstants.kDriveKinematics);
        this.trajectory =
            TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
                List.of(new Translation2d(1, 0), new Translation2d(2,0)),
                new Pose2d(3, 0,Rotation2d.fromDegrees(0)),
                config);
        this.thetaController =
            new ProfiledPIDController(
                Constants.SwerveConstants.AutoConstants.kPThetaController, 0, 0, Constants.SwerveConstants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        this.subsystem.setPose(trajectory.getInitialPose());
        SwerveControllerCommand swerveControllerCommand =
            new SwerveControllerCommand(
                this.trajectory,
                this.subsystem::getPose,
                Constants.SwerveConstants.DriveConstants.kDriveKinematics,
                new PIDController(Constants.SwerveConstants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.SwerveConstants.AutoConstants.kPYController, 0, 0),
                thetaController,
                this.subsystem::setModuleStates,
                this.subsystem);
        addCommands(

            new InstantCommand(
                () -> subsystem.setPose(trajectory.getInitialPose())
                ),
            swerveControllerCommand,
            new InstantCommand(
                () -> subsystem.stopModules()
                )
            );
    }
}
