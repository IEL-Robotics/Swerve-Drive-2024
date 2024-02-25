package frc.robot.commands.Pneumatic;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PneumaticSubsystem;

public class PistonClose extends Command {
    public PneumaticSubsystem pneumaticSubsystem;

    public PistonClose(PneumaticSubsystem pneumaticSubsystem) {
        this.pneumaticSubsystem = pneumaticSubsystem;
    }

    @Override
    public void initialize() {
    }

    public void reInit() {
    }

    @Override
    public void execute() {
        pneumaticSubsystem.pistonClose();
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
