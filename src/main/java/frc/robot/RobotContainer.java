package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SwerveConstants.AutoConstants;
import frc.robot.Constants.SwerveConstants.DriveConstants;
import frc.robot.Constants.SwerveConstants.OIConstants;
import frc.robot.commands.Swerve.ResetEncoder;
import frc.robot.commands.Swerve.SwerveDrive;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
  public final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

  private final ResetEncoder m_ResetEncoder = new ResetEncoder(swerveSubsystem);

  private final PS4Controller m_driverController = new PS4Controller(OperatorConstants.kDriverControllerPort);

  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(getTeleopCommand());

    configureBindings();
  }

  private void configureBindings() {
        new JoystickButton(m_driverController, 2).onTrue(m_ResetEncoder);
  }

  public Command getTeleopCommand() {
    
    //double betaAngle = ((Math.atan2(-(m_driverController.getRawAxis(0)),(m_driverController.getRawAxis(1)))));
    
    return new SwerveDrive(
        swerveSubsystem,
        () -> m_driverController.getRawAxis(OIConstants.kDriverYAxis),
        () -> m_driverController.getRawAxis(OIConstants.kDriverXAxis),
        () -> m_driverController.getRawAxis(OIConstants.kDriverRotAxis),
        () -> !(m_driverController.getL2Axis()>0.5)
    );

  }

  public Command getAutonomousCommand() {
    swerveSubsystem.zeroHeading();

    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared).setKinematics(DriveConstants.kDriveKinematics);
    
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                            new Pose2d(0, 0, new Rotation2d(0)),
                            List.of(
                              new Translation2d(0, 1),
                              new Translation2d(0, 2),
                              new Translation2d(0, 3)
                            ),
                            new Pose2d(0, 4, new Rotation2d(6.56)),
                            trajectoryConfig);

    PIDController xController = new PIDController(AutoConstants.kPXController, 0.01, 0.005);
    PIDController yController = new PIDController(AutoConstants.kPYController, 0.01, 0.005);
    ProfiledPIDController thetaController = new ProfiledPIDController(
                          AutoConstants.kPThetaController, 0.01, 0.005, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
      trajectory, swerveSubsystem::getPose, DriveConstants.kDriveKinematics, xController, yController, thetaController,
      swerveSubsystem::setModuleStates, swerveSubsystem);

    return new SequentialCommandGroup(
      new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory.getInitialPose())),
      swerveControllerCommand,
      new InstantCommand(() -> swerveSubsystem.stopModules())
    );
  }

  public void allValuesDisplay() {
    swerveSubsystem.allValuesDisplay();
    double betaAngle = ((Math.atan2(-(m_driverController.getRawAxis(0)),(m_driverController.getRawAxis(1)))));
  }
}
