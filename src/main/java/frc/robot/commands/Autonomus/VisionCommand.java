package frc.robot.commands.Autonomus;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Vision;

public class VisionCommand extends Command {

    Vision m_Vision;

    public VisionCommand(Vision m_Vision) {
        this.m_Vision = m_Vision;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double[] positions=m_Vision.getFieldPosition();
        int id=m_Vision.getTagID();
        SmartDashboard.putNumber("X pos", positions[0]);
        SmartDashboard.putNumber("Y pos", positions[1]);
        SmartDashboard.putNumber("angle pos", positions[2]);
        SmartDashboard.putString("Tag id", String.valueOf(id));
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
