package frc.robot.commands.Arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class PresetArm extends Command {
    
    public ArmSubsystem armSubsystem;
    double presetValue;

    public PIDController pidController = new PIDController(0.5, 0.0, 0.0);
    boolean myInit = true;

    public PresetArm(ArmSubsystem armSubsystem, double presetValue) {
        this.armSubsystem = armSubsystem;
        this.presetValue = presetValue;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
    }

    public void reInit() {
        pidController.setSetpoint(presetValue);
        pidController.setTolerance(30, 10);
        myInit = false;
    }

    @Override
    public void execute() {
        if(myInit){reInit();}
        armSubsystem.armSet(MathUtil.clamp(pidController.calculate(armSubsystem.getRightEncoderVal()), -1, 1));
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.armSet(0);
        myInit = true;
    }

    @Override
    public boolean isFinished() {
        return pidController.atSetpoint();
    }
}
