package frc.robot.commands.Vision;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class Aim extends Command {
    
    public ArmSubsystem armSubsystem;
    public VisionSubsystem visionSubsystem;
    private PS4Controller joystick;
    double presetValue;

    public PIDController pidController=new PIDController(5,1,0.0); // i cok iyi, range ekle sadece, ve arttir
    boolean myInit = true;

    public Aim(ArmSubsystem armSubsystem, VisionSubsystem visionSubsystem, PS4Controller joystick) {
        this.armSubsystem = armSubsystem;
        this.visionSubsystem = visionSubsystem;
        this.joystick = joystick;
        pidController.setIZone(0.15);
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
    }

    public void reInit() {
        presetValue = loop();
        pidController.setSetpoint(presetValue);
        //pidController.setTolerance(10, 15);
        myInit = false;
    }

    public double customSigmoid(double input) { // -1/3 -> Questionable -> Emel kiziyo
        double maxAngSpd = 0.85;
        return ((2*maxAngSpd)/(1+Math.pow(Math.E,((double)-1.0)*(input))))-(maxAngSpd);
    }

    @Override
    public void execute() {
        if(myInit){reInit();}
        System.out.println("AIM COOMAND RUNNING");
        //armSubsystem.armSet(customSigmoid(pidController.calculate(armSubsystem.getPotentiometerVoltage())));
        armSubsystem.armSet(4 * -MathUtil.clamp(pidController.calculate(armSubsystem.getPotentiometerVoltage()), -1, 1));
        System.out.println(MathUtil.clamp(pidController.calculate(armSubsystem.getPotentiometerVoltage()), -1, 1));
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("PRESET REACHED!!!");
        armSubsystem.armSet(0);
        myInit = true;
    }

    @Override
    public boolean isFinished() {
        //System.out.println(Math.abs(presetValue - armSubsystem.getPotentiometerVoltage()));
        return Math.abs(presetValue - armSubsystem.getPotentiometerVoltage()) < 0.005 || joystickInterference();
    }

    public double loop() {
        double[] positions = visionSubsystem.getFieldPosition();

        double distance = DriverStation.getAlliance().get() == DriverStation.Alliance.Red ? visionSubsystem.getDistanceForRed(positions[0], positions[1]) : visionSubsystem.getDistanceForBlue(positions[0], positions[1]);
        SmartDashboard.putNumber("RegressionPredict", regressionPredictPotentiometer(distance));
        return regressionPredictPotentiometer(distance);
    }

    public double regressionPredict(double distance){
        return -36.445 * Math.pow(distance, 3) + 205.97 * Math.pow(distance, 2) - 297.89 * Math.pow(distance, 1) - 1093.5;
    }

    public double regressionPredictPotentiometer(double distance) {
        return -(-0.0364 * Math.pow(distance, 3) + 0.206 * Math.pow(distance, 2) - 0.2979 * distance - 2.3075);

    }

    public boolean joystickInterference() {
        return Math.abs(joystick.getRawAxis(0)) > 0.25 || Math.abs(joystick.getRawAxis(1)) > 0.25;
    }

}

