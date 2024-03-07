package frc.robot.commands.Swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class ResetGyro extends Command {
  public SwerveSubsystem m_SwerveSubsystem;

  public ResetGyro(SwerveSubsystem m_SwerveSubsystem) {
    this.m_SwerveSubsystem = m_SwerveSubsystem;
    addRequirements();
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    m_SwerveSubsystem.zeroHeading();
    m_SwerveSubsystem.resetOdometer(new Pose2d(new Translation2d(0.0, 0.0), new Rotation2d(0.0)));
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
