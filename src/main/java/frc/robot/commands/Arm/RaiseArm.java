package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class RaiseArm extends Command {
    public ArmSubsystem armSubsystem;

    public RaiseArm(ArmSubsystem armSubsystem) {
        this.armSubsystem = armSubsystem;
        addRequirements();
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        armSubsystem.armSet(0.6);
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