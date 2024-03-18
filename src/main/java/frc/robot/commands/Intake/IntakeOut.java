package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeOut extends Command {
    
    public IntakeSubsystem intakeSubsystem;

    //private final Supplier<Double> sSpdFunction;

    public IntakeOut(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        addRequirements();
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        intakeSubsystem.runMotorsInverted();
        System.out.println("IntakeOut");
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stopMotors();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
