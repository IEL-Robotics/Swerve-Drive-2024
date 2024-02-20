package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.SwerveSubsystem;

public class lowerArm extends Command {
    public Arm armSubsystem;

    public lowerArm(Arm armSubsystemInput) {
        armSubsystem = armSubsystemInput;
        addRequirements();
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        armSubsystem.armSet(-0.75);
        System.out.println("NegatifExecut");
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.armSet(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
