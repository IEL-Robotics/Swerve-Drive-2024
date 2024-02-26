package frc.robot.commands.Vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class ExperimentalSetStart extends Command {

    VisionSubsystem visionSubsystem;
    SwerveSubsystem swerveSubsystem;

    public ExperimentalSetStart(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        addRequirements(visionSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        swerveSubsystem.resetOdometer(new Pose2d(new Translation2d(5.0, 5.0), new Rotation2d(0)));
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
