package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class Delay extends Command {
    
    private final double delaySeconds;
    private double startTime;
    private boolean isDone;

    public Delay(double delaySeconds) {
        this.delaySeconds = delaySeconds;
        addRequirements();
    }

    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
        isDone = false;
    }

    @Override
    public void execute() {
        double currentTime = Timer.getFPGATimestamp();
        if (currentTime - startTime >= delaySeconds) {
            isDone = true;
        }

        System.out.println("Delay CMD Running");
    }

    @Override
    public void end(boolean interrupted){
    }

    @Override
    public boolean isFinished() {
        return isDone;
    }
}
