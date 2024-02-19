package frc.robot.commands.Swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class rotateToSpecificDeg extends Command {

    public SwerveSubsystem swerveSubsystem;
    public PIDController pidController = new PIDController(0.25, 0.25, 0.005);
    ChassisSpeeds chassisSpeeds;

    double error, output, desiredAngle, currentAngle, realDesiredAngle;
    int kConstant;
    boolean myInit = true;

  public rotateToSpecificDeg(SwerveSubsystem swerveSubsystem, double desiredAngle) {
    this.swerveSubsystem = swerveSubsystem;
    this.desiredAngle = desiredAngle;
    pidController.enableContinuousInput(-180, 180);
    addRequirements();
  }

  @Override
  public void initialize() {
    kConstant = (int) swerveSubsystem.getHeadingEndless() / 360;
    realDesiredAngle = (desiredAngle) + (360*kConstant);
    SmartDashboard.putNumber("RDAngle", realDesiredAngle);
  }

  @Override
  public void execute() { //sapma yazilimsal mi anlamak icin turningspd 0 feedle, sonra sur, rotasyonu kapa yani
//pid i bitirmesene bi
    if(myInit){
      kConstant = (int) swerveSubsystem.getHeadingEndless() / 360;
      realDesiredAngle = (desiredAngle) + (360*kConstant);
      SmartDashboard.putNumber("RDAngle", realDesiredAngle);
      myInit = false;
    }

    currentAngle = swerveSubsystem.getHeadingEndless();
    output = pidController.calculate(currentAngle, realDesiredAngle);
    error = realDesiredAngle - currentAngle;
    
    SmartDashboard.putNumber("Error temp", error);
    SmartDashboard.putNumber("Output temp", swerveSubsystem.customSigmoid(output));

    chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 
    -swerveSubsystem.customSigmoid(output), swerveSubsystem.getRotation2d());

    SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

    swerveSubsystem.setModuleStates(moduleStates);
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

    //return false;
  }
}
