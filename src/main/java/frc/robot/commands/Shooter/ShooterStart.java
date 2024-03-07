package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterStart extends Command {
    
    public ShooterSubsystem shooterSubsystem;
    public double Spd;

    public ShooterStart(ShooterSubsystem shooterSubsystem, double Spd) {
        this.shooterSubsystem = shooterSubsystem;
        this.Spd = Spd;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        shooterSubsystem.runMotors(Spd);
        shooterSubsystem.debug();
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return true;
    }
}
