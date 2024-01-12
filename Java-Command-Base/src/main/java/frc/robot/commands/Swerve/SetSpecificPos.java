package frc.robot.commands.Swerve;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.SwerveSubsystem;

public class SetSpecificPos extends CommandBase {
  public SwerveSubsystem m_SwerveSubsystem;
  public SwerveModule lf_SwerveModule, rf_SwerveModule,rb_SwerveModule;

  // public double setPoint;

  public final Supplier<Double> xSpdFunc;
  public final Supplier<Double> ySpdFunc;

  public PIDController tempPID = new PIDController(0.5, 0.01, 0.0);

  public double offSet[];

  public SetSpecificPos(SwerveSubsystem m_SwerveSubsystem,
                        SwerveModule lf_SwerveModule,
                        SwerveModule rf_SwerveModule,
                        SwerveModule rb_SwerveModule,
                        Supplier<Double> xSpdFunc, 
                        Supplier<Double> ySpdFunc, 
                        double[] offSet) {

    this.m_SwerveSubsystem = m_SwerveSubsystem;
    this.lf_SwerveModule = lf_SwerveModule;
    this.rf_SwerveModule = rf_SwerveModule;
    this.rb_SwerveModule = rb_SwerveModule;

    this.xSpdFunc = xSpdFunc;
    this.ySpdFunc = ySpdFunc;

    this.offSet = offSet;

    addRequirements(m_SwerveSubsystem);
  }

  @Override
  public void initialize() {
    }

  @Override
  public void execute() {
    double betaAngle = (Math.atan2(-xSpdFunc.get(),ySpdFunc.get()));

    if(Math.abs(xSpdFunc.get()) > 0.15 || Math.abs(ySpdFunc.get()) > 0.15){ 
      lf_SwerveModule.turningMotor.set(tempPID.calculate(lf_SwerveModule.getAbsEncRad(), (betaAngle + offSet[0])));
      rf_SwerveModule.turningMotor.set(tempPID.calculate(rf_SwerveModule.getAbsEncRad(), (betaAngle + offSet[1])));
      rb_SwerveModule.turningMotor.set(tempPID.calculate(rb_SwerveModule.getAbsEncRad(), (betaAngle + offSet[2])));
    } // How to Optimize
    else{
      lf_SwerveModule.turningMotor.set(0);
      rf_SwerveModule.turningMotor.set(0);
      rb_SwerveModule.turningMotor.set(0);
    }
  }
    
  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    //SmartDashboard.putNumber("SAPMA", Math.abs(m_SwerveModule.getAbsEncRad() - (setPointFunc.get() + offSet)));
    // if(Math.abs(m_SwerveModule.getAbsEncRad() - (setPoint + offSet)) < 0.03){
    //   return true;
    // }
    // System.out.println("LOG");
    return false;
  }
}
