package frc.robot.commands.Swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.KinematicsConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class RotateToSpecificAngle extends Command {

    public SwerveSubsystem swerveSubsystem;
    public PIDController pidController = new PIDController(0.25, 0.25, 0.005);
    ChassisSpeeds chassisSpeeds;

    double error, output, desiredAngle, currentAngle, realDesiredAngle;
    int kConstant;
    boolean myInit = true;

  public RotateToSpecificAngle(SwerveSubsystem swerveSubsystem, double desiredAngle) {
    this.swerveSubsystem = swerveSubsystem;
    this.desiredAngle = desiredAngle;
    pidController.enableContinuousInput(-180, 180);
    addRequirements();
  }

  @Override
  public void initialize() {
    kConstant = (int) swerveSubsystem.getRobotHeadingEndless() / 360;
    realDesiredAngle = (desiredAngle) + (360*kConstant);
  }

  @Override
  public void execute() { //sapma yazilimsal mi anlamak icin turningspd 0 feedle, sonra sur, rotasyonu kapa yani//pid i bitirmesene bi
    if(myInit){
      kConstant = (int) swerveSubsystem.getRobotHeadingEndless() / 360;
      realDesiredAngle = (desiredAngle) + (360*kConstant);
      myInit = false;
    }

    currentAngle = swerveSubsystem.getRobotHeadingEndless();
    output = pidController.calculate(currentAngle, realDesiredAngle);
    error = realDesiredAngle - currentAngle;

    chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 
    -swerveSubsystem.customSigmoid(output), swerveSubsystem.getRotation2d());

    SwerveModuleState[] moduleStates = KinematicsConstants.KINEMATICS_DRIVE_CHASSIS.toSwerveModuleStates(chassisSpeeds);

    swerveSubsystem.setModuleStates(moduleStates, false); //?
    swerveSubsystem.updateSayac(); 

    System.out.println("Executing rotateToSpecificDeg");
  }

  @Override
  public void end(boolean interrupted) {
    myInit = true;
    swerveSubsystem.stopModules();
    System.out.println("FINISHED rotateToSpecificDeg");
  }

  @Override
  public boolean isFinished() {
    if(Math.abs(error) < 5){return true;}
    else{return false;}
  }
}
