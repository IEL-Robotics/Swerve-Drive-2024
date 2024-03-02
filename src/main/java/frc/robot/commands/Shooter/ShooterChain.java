package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Pneumatic.PistonClose;
import frc.robot.commands.Pneumatic.PistonOpen;

public class ShooterChain extends SequentialCommandGroup {

    public ShooterChain(ShooterStart SHOOTER_START, ShooterStop SHOOTER_STOP,
            PistonClose PISTON_CLOSE, PistonOpen PISTON_OPEN) {

        addCommands(
            SHOOTER_START, new WaitCommand(1),
            PISTON_OPEN, new WaitCommand(1),
            PISTON_CLOSE,
            SHOOTER_STOP
        );
    }

}
