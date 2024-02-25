package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

public class Debug extends Command {

    public Debug() {
        addRequirements();
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        System.out.println("Debug Cmd Running");
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("DEBUG CMD ENDED");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
