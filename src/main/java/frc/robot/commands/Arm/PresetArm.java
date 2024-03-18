package frc.robot.commands.Arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class PresetArm extends Command {
    
    public ArmSubsystem armSubsystem;
    double presetValue;

    //public PIDController pidController = new PIDController(0.05, 0.015, 0.0); // i cok iyi, range ekle sadece, ve arttir
                                                  
   
    public PIDController pidController=new PIDController(5,1,0.0); // imetin pid
    //public PIDController pidController=new PIDController(0.05,0.015 ,0.004); //ben pid
    boolean myInit = true;

    public PresetArm(ArmSubsystem armSubsystem, double presetValue) {
        this.armSubsystem = armSubsystem;
        this.presetValue = presetValue;
        pidController.setIZone(0.2);
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
    }

    public void reInit() {
        pidController.setSetpoint(presetValue);
        //pidController.setTolerance(10, 15);
        myInit = false;
    }

    public double customSigmoid(double input) { // -1/3 -> Questionable -> Emel kiziyo
        double maxAngSpd = 0.75;
        return ((2*maxAngSpd)/(1+Math.pow(Math.E,((double)-1.0)*(input))))-(maxAngSpd);
    }

    @Override
    public void execute() {
        //System.out.println("ArmToPreset Blocking, Output: " + pidController.calculate(armSubsystem.getPotentiometerVoltage()));
        if(myInit){reInit();}
        armSubsystem.armSet(-MathUtil.clamp(pidController.calculate(armSubsystem.getPotentiometerVoltage()), -1, 1));
        //armSubsystem.armSet(customSigmoid(pidController.calculate(armSubsystem.getPotentiometerVoltage())));
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("PRESET REACHED!!!");
        armSubsystem.armSet(0);
        myInit = true;
    }

    @Override
    public boolean isFinished() {
    System.out.println("Error is " + Math.abs(presetValue - armSubsystem.getPotentiometerVoltage()));
    return Math.abs(presetValue - armSubsystem.getPotentiometerVoltage()) < 0.01; 
    //return pidController.atSetpoint()
    }
}
