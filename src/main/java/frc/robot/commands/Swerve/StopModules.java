package frc.robot.commands.Swerve;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class StopModules extends Command {

    private final SwerveSubsystem subsystem;

    public StopModules(SwerveSubsystem subsystem){
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        subsystem.stopModules();
        System.out.println("StopModules Ran");
    }

    @Override
    public void end(boolean interrupted) {
    }
    
    @Override
    public boolean isFinished() {
        return true;
    }
}
