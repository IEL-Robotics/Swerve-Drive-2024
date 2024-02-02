package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SwerveConstants.OIConstants;
import frc.robot.commands.Swerve.ResetEncoder;
import frc.robot.commands.Swerve.SetSpecificPos;
import frc.robot.commands.Swerve.SwerveDrive;
import frc.robot.subsystems.SwerveSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import frc.robot.Constants.SwerveConstants.DriveConstants;
import frc.robot.Constants.SwerveConstants.AutoConstants;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Autonumous.AutoDrive;

public class RobotContainer {
  public final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

  private final ResetEncoder m_ResetEncoder = new ResetEncoder(swerveSubsystem);

  private final PS4Controller m_driverController = new PS4Controller(OperatorConstants.kDriverControllerPort);

  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(getTeleopCommand());

    configureBindings();
  }

  private void configureBindings() {
        //new JoystickButton(m_driverController, 2).whenActive(() -> swerveSubsystem.zeroHeading());
        new JoystickButton(m_driverController, 2).onTrue(m_ResetEncoder);
        // new JoystickButton(m_driverController, 4).onTrue(new ParallelCommandGroup( new SetSpecificPos(swerveSubsystem.frontRight, 0.25, 0),
        // new SetSpecificPos(swerveSubsystem.backRight, 0.25, -1.5),
        // new SetSpecificPos(swerveSubsystem.frontLeft, 0.25, 1))); //problematic, 0 -> 3.14
  }

  public Command getAutonomousCommand() {
    PathPlannerPath path = PathPlannerPath.fromPathFile("Test Path");
    /*return new FollowPathHolonomic(
      path,
      swerveSubsystem::getPose,
      swerveSubsystem::getRobotRelativeSpeeds,
      swerveSubsystem::driveRobotRelative,
      new HolonomicPathFollowerConfig(
                        new PIDConstants(0.1, 0.0, 0.005),
                        new PIDConstants(0.1, 0.0, 0.005), 
                        AutoConstants.kMaxSpeedMetersPerSecond, 
                        Math.hypot(DriveConstants.kTrackWidth / 2, DriveConstants.kWheelBase / 2),
                        new ReplanningConfig()                
                        ),
                () -> {
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                swerveSubsystem 
        );*/
      return Commands.runOnce(() -> swerveSubsystem.resetOdometry(path.getPreviewStartingHolonomicPose())).andThen(AutoBuilder.followPath(path));
  }

  public Command getTeleopCommand() {
    
    double betaAngle = ((Math.atan2(-(m_driverController.getRawAxis(0)),(m_driverController.getRawAxis(1)))));
    
    return new SwerveDrive(
        swerveSubsystem,
        () -> m_driverController.getRawAxis(OIConstants.kDriverYAxis),
        () -> m_driverController.getRawAxis(OIConstants.kDriverXAxis),
        () -> m_driverController.getRawAxis(OIConstants.kDriverRotAxis),
        () -> !(m_driverController.getL2Axis()>0.5)
    );

    // double[] offsets = {1, 0.2, -1.5};
    // return new SetSpecificPos(
    //   swerveSubsystem,
    //   swerveSubsystem.frontLeft,
    //   swerveSubsystem.frontRight,
    //   swerveSubsystem.backRight,
    //   () -> m_driverController.getRawAxis(0),
    //   () -> - m_driverController.getRawAxis(1),
    //   offsets);
}

  public void allValuesDisplay() {
    swerveSubsystem.allValuesDisplay();
    
    double betaAngle = ((Math.atan2(-(m_driverController.getRawAxis(0)),(m_driverController.getRawAxis(1)))));
    SmartDashboard.putNumber("Beta as Degree", betaAngle);
    // SmartDashboard.putNumber("Y", m_driverController.getRawAxis(1));
    // SmartDashboard.putNumber("X", m_driverController.getRawAxis(0));
  }
  
}
