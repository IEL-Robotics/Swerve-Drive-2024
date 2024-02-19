package frc.robot;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SwerveConstants.AutoConstants;
import frc.robot.Constants.SwerveConstants.DriveConstants;
import frc.robot.Constants.SwerveConstants.OIConstants;
import frc.robot.commands.Autonomus.TrajectoryDrive1;
import frc.robot.commands.Swerve.ResetGyro;
import frc.robot.commands.Swerve.SwerveDrive;
import frc.robot.commands.Swerve.rotateThisMuch;
import frc.robot.commands.Swerve.rotateToSpecificDeg;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
  public final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

  private final ResetGyro m_ResetGyro = new ResetGyro(swerveSubsystem);

  private final PS4Controller m_driverController = new PS4Controller(OperatorConstants.kDriverControllerPort);

  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(getTeleopCommand());

    configureBindings();
  }

  private void configureBindings() {
    //Cross
        new JoystickButton(m_driverController, 2).onTrue(m_ResetGyro);
    //Circle
        new JoystickButton(m_driverController, 3).onTrue(new rotateThisMuch(swerveSubsystem, 90));
    //Square
        new JoystickButton(m_driverController, 1).onTrue(new rotateThisMuch(swerveSubsystem, -90));
    //Triangle
        new JoystickButton(m_driverController, 4).onTrue(new rotateToSpecificDeg(swerveSubsystem, 72));
  }

  public Command getTeleopCommand() {
    
    //double betaAngle = ((Math.atan2(-(m_driverController.getRawAxis(0)),(m_driverController.getRawAxis(1)))));
    
    return new SwerveDrive(
        swerveSubsystem,
        () -> m_driverController.getRawAxis(OIConstants.kDriverYAxis),
        () -> m_driverController.getRawAxis(OIConstants.kDriverXAxis),
        () -> m_driverController.getRawAxis(OIConstants.kDriverRotAxis),
        () -> !(m_driverController.getRawButtonReleased(14))
    );

  }

  public Command getAutonomousCommand() {
    PathPlannerPath path = PathPlannerPath.fromPathFile("Test Path");
    return Commands.runOnce(() -> swerveSubsystem.resetOdometry(path.getPreviewStartingHolonomicPose())).andThen(AutoBuilder.followPath(path));
  }

  public void allValuesDisplay() {
    swerveSubsystem.allValuesDisplay();
    double betaAngle = ((Math.atan2(-(m_driverController.getRawAxis(0)),(m_driverController.getRawAxis(1)))));
  }
}
