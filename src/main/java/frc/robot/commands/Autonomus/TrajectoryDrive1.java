package frc.robot.commands.Autonomus;
import frc.robot.Constants.SwerveConstants.AutoConstants;
import frc.robot.Constants.SwerveConstants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class TrajectoryDrive1 extends SequentialCommandGroup{
    SwerveSubsystem subsystem;
    public TrajectoryDrive1(SwerveSubsystem swerveSubsystem){
    swerveSubsystem.zeroHeading();

    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared).setKinematics(DriveConstants.kDriveKinematics);
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                            new Pose2d(0*3.6, 0*3.6, new Rotation2d(0)),
                            List.of(
                              new Translation2d(0*3.6, -1.5*3.6),
                              new Translation2d(2*3.6, -1.5*3.6),
                              new Translation2d(2*3.6, 0*3.6)
                            ),
                            new Pose2d(0*3.6, 0*3.6, Rotation2d.fromDegrees(0)),
                            trajectoryConfig);

    PIDController xController = new PIDController(AutoConstants.kPXController, 0.01, 0.005);
    PIDController yController = new PIDController(AutoConstants.kPYController, 0.01, 0.005);
    /*ProfiledPIDController thetaController = new ProfiledPIDController(
                          2.75, 0.1, 0.0035, AutoConstants.kThetaControllerConstraints);
                          */

    ProfiledPIDController thetaController = new ProfiledPIDController(
                          0.75, 0.15, 0.0035, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
      trajectory, swerveSubsystem::getPose, DriveConstants.kDriveKinematics, xController, yController, thetaController,
      swerveSubsystem::setModuleStates, swerveSubsystem);
      addCommands(new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory.getInitialPose())),
      swerveControllerCommand,
      new InstantCommand(() -> swerveSubsystem.stopModules()));
    }
}
