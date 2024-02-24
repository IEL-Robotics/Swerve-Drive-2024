package frc.robot.commands.Swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.KinematicsConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class RotateThisMuch extends Command {

    public SwerveSubsystem swerveSubsystem;
    public PIDController pidController = new PIDController(0.25, 0.25, 0.005);
    ChassisSpeeds chassisSpeeds;

    double error, output, initialAngle, desiredAngle, currentAngle;
    boolean myInit = true;

  public RotateThisMuch(SwerveSubsystem swerveSubsystem, double desiredAngle) {
    this.swerveSubsystem = swerveSubsystem;
    this.desiredAngle = desiredAngle;
    addRequirements();
  }

  @Override
  public void initialize() {
    initialAngle = swerveSubsystem.getRobotHeadingEndless();
  }

  @Override
  public void execute() {
    if(myInit){initialAngle = swerveSubsystem.getRobotHeadingEndless();myInit = false;}
  
    currentAngle = swerveSubsystem.getRobotHeadingEndless();
    output = pidController.calculate(currentAngle, desiredAngle + initialAngle);
    error = (desiredAngle + initialAngle) - currentAngle;

    chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 
    -swerveSubsystem.customSigmoid(output), swerveSubsystem.getRotation2d());

    SwerveModuleState[] moduleStates = KinematicsConstants.KINEMATICS_DRIVE_CHASSIS.toSwerveModuleStates(chassisSpeeds);

    swerveSubsystem.setModuleStates(moduleStates, false); // ?
    swerveSubsystem.updateSayac(); 

    System.out.println("Executing rotateThisMuch");
  }
    
  @Override
  public void end(boolean interrupted) {
    myInit = true;
    swerveSubsystem.stopModules();
    System.out.println("FINISHED rotateThisMuch");
  }

  @Override
  public boolean isFinished() {
    if(Math.abs(error) < 2.5){return true;}
    else{return false;}

    //return false;
  }
}
