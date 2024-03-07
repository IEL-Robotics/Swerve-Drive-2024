package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class LowerArm extends Command {
    public ArmSubsystem armSubsystem;

    public LowerArm(ArmSubsystem armSubsystem) {
        this.armSubsystem = armSubsystem;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        // -0.5 idi
        armSubsystem.armSet(-0.8);
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
