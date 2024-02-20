package frc.robot;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SwerveConstants.OIConstants;
import frc.robot.commands.Arm.lowerArm;
import frc.robot.commands.Arm.raiseArm;
//import frc.robot.commands.Autonomus.TrajectoryDrive1;
import frc.robot.commands.Swerve.ResetGyro;
import frc.robot.commands.Swerve.SwerveDrive;
import frc.robot.commands.Swerve.rotateThisMuch;
import frc.robot.commands.Swerve.rotateToSpecificDeg;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.SwerveSubsystem;
//import frc.robot.subsystems.Vision;

public class RobotContainer {
  public final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  public final Arm armSubsystem =  new Arm();
  //public final Vision visionSubsystem = new Vision();

  private final ResetGyro m_ResetGyro = new ResetGyro(swerveSubsystem);
  private final raiseArm m_RaiseArm = new raiseArm(armSubsystem);
  private final lowerArm m_LowerArm = new lowerArm(armSubsystem);

  private final PS4Controller m_driverController = new PS4Controller(OperatorConstants.kDriverControllerPort);

  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(getTeleopCommand());
    //visionSubsystem.setDefaultCommand(getAutonomousCommand());

    configureBindings();
  }

  private void configureBindings() {
    // Cross
    new JoystickButton(m_driverController, 2).onTrue(m_ResetGyro);
    // Circle
    new JoystickButton(m_driverController, 3).onTrue(new rotateThisMuch(swerveSubsystem, 90));
    // Square
    new JoystickButton(m_driverController, 1).onTrue(new rotateThisMuch(swerveSubsystem, -90));
    // Triangle
    new JoystickButton(m_driverController, 4).onTrue(new rotateToSpecificDeg(swerveSubsystem, 72));
    new JoystickButton(m_driverController, 5).whileTrue(m_RaiseArm);
    new JoystickButton(m_driverController, 6).whileTrue(m_LowerArm);
  }

  public Command getTeleopCommand() {
    // double betaAngle =
    // ((Math.atan2(-(m_driverController.getRawAxis(0)),(m_driverController.getRawAxis(1)))));

    return new SwerveDrive(
        swerveSubsystem,
        () -> m_driverController.getRawAxis(OIConstants.kDriverYAxis),
        () -> m_driverController.getRawAxis(OIConstants.kDriverXAxis),
        () -> m_driverController.getRawAxis(OIConstants.kDriverRotAxis),
        () -> !(m_driverController.getRawButtonReleased(14)));

  }

  // public Command getAutonomousCommand() {
  //   double[] xCor1 = { 1, 2, 3 };
  //   double[] yCor1 = { 0, 0, 0 };
  //   double[] xCor2 = { 3, 3, 3 };
  //   double[] yCor2 = { 1, 2, 3 };
  //   double[] xCor3 = { 3, 4, 0 };
  //   double[] yCor3 = { 3, 4, 0 };

  //   TrajectoryDrive1 commandTraj1 = new TrajectoryDrive1(swerveSubsystem, visionSubsystem, xCor1, yCor1, 0., true);
  //   TrajectoryDrive1 commandTraj2 = new TrajectoryDrive1(swerveSubsystem, visionSubsystem, xCor2, yCor2, 0., false);
  //   TrajectoryDrive1 commandTraj3 = new TrajectoryDrive1(swerveSubsystem, visionSubsystem, xCor3, yCor3, 0., false);

  //   return new SequentialCommandGroup(commandTraj1, commandTraj2, commandTraj3);
  // }

  public Command getAutonomousCommand() {
    return null;
  }

  public void allValuesDisplay() {
    swerveSubsystem.allValuesDisplay();
  }
}