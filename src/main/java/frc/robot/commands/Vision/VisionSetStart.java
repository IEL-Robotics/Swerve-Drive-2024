package frc.robot.commands.Vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class VisionSetStart extends Command {
    
    VisionSubsystem visionSubsystem;
    SwerveSubsystem swerveSubsystem;

    public VisionSetStart(VisionSubsystem visionSubsystem, SwerveSubsystem swerveSubsystem){
        this.swerveSubsystem = swerveSubsystem;
        this.visionSubsystem = visionSubsystem;
        addRequirements(visionSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double[] positions = visionSubsystem.getFieldPosition();
        swerveSubsystem.resetOdometer(new Pose2d(new Translation2d(positions[0], positions[1]), new Rotation2d(Math.toRadians(positions[2]))));
        //swerveSubsystem.resetOdometer(new Pose2d(new Translation2d(1.28, 5.55), new Rotation2d(0)));
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Experimental Set Start RUN");
    }

    @Override
    public boolean isFinished() {
        return visionSubsystem.gotDataIndeed();
    }
}
