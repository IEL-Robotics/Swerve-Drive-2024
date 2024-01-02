package frc.robot.commands.Swerve;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

public class ResetEncoder extends CommandBase {
  public SwerveSubsystem m_SwerveSubsystem;

  public ResetEncoder(SwerveSubsystem m_SwerveSubsystem) {
    this.m_SwerveSubsystem = m_SwerveSubsystem;
    addRequirements();
  }

  @Override
    public void initialize() {
    }

    @Override
    public void execute() {
      m_SwerveSubsystem.zeroHeading();
    }
    @Override
    public void end(boolean interrupted) {
    }

  @Override
  public boolean isFinished() {
    return true;
  }
}
