package frc.robot.commands.Arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class PresetArm extends Command {
    
    public ArmSubsystem armSubsystem;
    double presetValue;

    public PIDController pidController = new PIDController(0.05, 0.015, 0.0); // i cok iyi, range ekle sadece, ve arttir
    boolean myInit = true;

    public PresetArm(ArmSubsystem armSubsystem, double presetValue) {
        this.armSubsystem = armSubsystem;
        this.presetValue = presetValue;
        pidController.setIZone(25);
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
    }

    public void reInit() {
        pidController.setSetpoint(presetValue);
        pidController.setTolerance(5, 10);
        myInit = false;
    }

    public double customSigmoid(double input) { // -1/3 -> Questionable -> Emel kiziyo
        double maxAngSpd = 1;
        return ((2*maxAngSpd)/(1+Math.pow(Math.E,((double)-1.0)*(input))))-(maxAngSpd);
    }

    @Override
    public void execute() {
        if(myInit){reInit();}
        //armSubsystem.armSet(MathUtil.clamp(pidController.calculate(armSubsystem.getRightEncoderVal()), -0.65, 0.65));
        armSubsystem.armSet(customSigmoid(pidController.calculate(armSubsystem.getRightEncoderVal())));
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("PRESET REACHED!!!");
        armSubsystem.armSet(0);
        myInit = true;
    }

    @Override
    public boolean isFinished() {
        return pidController.atSetpoint();
    }
}
