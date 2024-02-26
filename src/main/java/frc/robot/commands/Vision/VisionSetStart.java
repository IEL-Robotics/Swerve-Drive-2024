package frc.robot.commands.Vision;

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
        swerveSubsystem.resetOdometer(visionSubsystem.getPoseOfRobot());
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
