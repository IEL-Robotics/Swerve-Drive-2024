package frc.robot.commands.Vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class ExperimentalSetStart extends Command {

    SwerveSubsystem swerveSubsystem;

    public ExperimentalSetStart(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        swerveSubsystem.resetOdometer(new Pose2d(new Translation2d(8.0, 0.0), new Rotation2d(0)));
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Experimental Set Start RUN");
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
