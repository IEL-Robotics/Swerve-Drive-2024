package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeStop extends Command {
    
    public IntakeSubsystem intakeSubsystem;

    //private final Supplier<Double> sSpdFunction;

    public IntakeStop(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        addRequirements();
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        intakeSubsystem.stopMotors();
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
