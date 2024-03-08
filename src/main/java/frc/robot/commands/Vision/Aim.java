package frc.robot.commands.Vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class Aim extends Command {
    
    public ArmSubsystem armSubsystem;
    public VisionSubsystem visionSubsystem;
    private PS4Controller joystick;
    double presetValue;

    public PIDController pidController=new PIDController(0.05,0.01,0.0); // i cok iyi, range ekle sadece, ve arttir
    boolean myInit = true;

    public Aim(ArmSubsystem armSubsystem, VisionSubsystem visionSubsystem, PS4Controller joystick) {
        this.armSubsystem = armSubsystem;
        this.visionSubsystem = visionSubsystem;
        this.joystick = joystick;
        pidController.setIZone(20);
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
    }

    public void reInit() {
        pidController.setSetpoint(loop());
        pidController.setTolerance(5, 10);
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
        armSubsystem.armSet(customSigmoid(pidController.calculate(armSubsystem.getLeftEncoderVal())));
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("PRESET REACHED!!!");
        armSubsystem.armSet(0);
        myInit = true;
    }

    @Override
    public boolean isFinished() {
        return pidController.atSetpoint() || joystickInterference();
    }

    public double loop() {
        double[] positions = visionSubsystem.getFieldPosition();
        // if(Math.abs(5.55 - positions[1]) > 0.4){
        //     SmartDashboard.putNumber("RegressionPredict", xyRegressionPredict(Math.abs(DriverStation.getAlliance().get() == DriverStation.Alliance.Blue ? -0.04 - positions[0] : 16.58 - positions[0]), Math.abs(5.55 - positions[1])));
        //     return xyRegressionPredict(Math.abs(DriverStation.getAlliance().get() == DriverStation.Alliance.Red ? -0.04 - positions[0] : 16.58 - positions[0]), Math.abs(5.55 - positions[1]));
        // }
        // else{
        //     double distance = DriverStation.getAlliance().get() == DriverStation.Alliance.Blue ? visionSubsystem.getDistanceForRed(positions[0], positions[1]) : visionSubsystem.getDistanceForBlue(positions[0], positions[1]);
        //     SmartDashboard.putNumber("RegressionPredict", regressionPredict(distance));
        //     return regressionPredict(distance);
        // }

        double distance = DriverStation.getAlliance().get() == DriverStation.Alliance.Red ? visionSubsystem.getDistanceForRed(positions[0], positions[1]) : visionSubsystem.getDistanceForBlue(positions[0], positions[1]);
        SmartDashboard.putNumber("RegressionPredict", regressionPredict(distance));
        return regressionPredict(distance);
    }

    public double regressionPredict(double distance){
        //return (-36.8 * Math.pow(distance, 3) + 216.8 * Math.pow(distance, 2) - 299 * distance + 1105.6);
        //return (36.074 * Math.pow(distance,4) - 346.23 * Math.pow(distance,3) + 1152.4 * Math.pow(distance,2) - 1489.9 * distance + 1705.5) * 0.63;
        //return (39.765 * Math.pow(distance,4) - 379.45 * Math.pow(distance,3) + 1274.2 * Math.pow(distance,2) - 1676 * Math.pow(distance,1) + 917.45) * 0.63;
        //return (39.765 * Math.pow(distance,4) - 379.45 * Math.pow(distance,3) + 1274.2 * Math.pow(distance,2) - 1676 * Math.pow(distance,1) - 318.55);
        return -36.445 * Math.pow(distance, 3) + 205.97 * Math.pow(distance, 2) - 297.89 * Math.pow(distance, 1) - 1093.5;


    }

    public double xyRegressionPredict(double x, double y){
        return 106.5*x + 37*y + 869;
    }

    public boolean joystickInterference() {
        return Math.abs(joystick.getRawAxis(0)) > 0.15 || Math.abs(joystick.getRawAxis(1)) > 0.15;
    }

}

