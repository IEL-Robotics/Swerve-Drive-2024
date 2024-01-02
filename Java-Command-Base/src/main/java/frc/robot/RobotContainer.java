package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SwerveConstants.OIConstants;
import frc.robot.commands.Swerve.ResetEncoder;
import frc.robot.commands.Swerve.SwerveDrive;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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
  }

  public Command getAutonomousCommand() {
    return null;
  }

  public Command getTeleopCommand() {
    return new SwerveDrive(
        swerveSubsystem,
        () -> -m_driverController.getRawAxis(OIConstants.kDriverYAxis),
        () -> m_driverController.getRawAxis(OIConstants.kDriverXAxis),
        () -> m_driverController.getRawAxis(OIConstants.kDriverRotAxis),
        () -> !(m_driverController.getL2Axis()>0.5)
    );
}
}
