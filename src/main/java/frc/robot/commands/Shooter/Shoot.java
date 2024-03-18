package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class Shoot extends Command {
    
    public ShooterSubsystem shooterSubsystem;

    //private final Supplier<Double> sSpdFunction;

    public Shoot(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        //this.sSpdFunction = sSpdFunciton;
        addRequirements();
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        // double sSpd = sSpdFunction.get();
        // sSpd = sSpd > 0.2 ? sSpd : 0.0;
        // System.out.println(sSpd);
        shooterSubsystem.debug();
        shooterSubsystem.runMotors(1);
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.runMotors(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
