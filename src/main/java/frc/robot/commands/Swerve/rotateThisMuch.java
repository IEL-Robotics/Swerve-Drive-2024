package frc.robot.commands.Swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class rotateThisMuch extends Command {

    public SwerveSubsystem swerveSubsystem;
    public PIDController pidController = new PIDController(0.25, 0.25, 0.005);
    ChassisSpeeds chassisSpeeds;

    double error, output, initialAngle, desiredAngle, currentAngle;
    boolean myInit = true;

  public rotateThisMuch(SwerveSubsystem swerveSubsystem, double desiredAngle) {
    this.swerveSubsystem = swerveSubsystem;
    this.desiredAngle = desiredAngle;
    addRequirements();
  }

  @Override
  public void initialize() {
    initialAngle = swerveSubsystem.getHeadingEndless();
  }

  @Override
  public void execute() {
    if(myInit){initialAngle = swerveSubsystem.getHeadingEndless();myInit = false;System.out.println("myInit CHANGED " + initialAngle);}
  
    currentAngle = swerveSubsystem.getHeadingEndless();
    output = pidController.calculate(currentAngle, desiredAngle + initialAngle);
    error = (desiredAngle + initialAngle) - currentAngle;


    SmartDashboard.putNumber("Error", error);
    SmartDashboard.putNumber("RawOutput", output);
    SmartDashboard.putNumber("SigmoidOutput", swerveSubsystem.customSigmoid(output));

    chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 
    -swerveSubsystem.customSigmoid(output), swerveSubsystem.getRotation2d());

    SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

    swerveSubsystem.setModuleStates(moduleStates);
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
