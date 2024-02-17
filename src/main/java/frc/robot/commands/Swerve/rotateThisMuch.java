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
    public PIDController pidController = new PIDController(0.3, 0.0015, 0.00);
    ChassisSpeeds chassisSpeeds;

    double error, output, initialAngle, desiredAngle;

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

    error = (desiredAngle + initialAngle) - swerveSubsystem.getHeadingEndless();
    output = pidController.calculate(swerveSubsystem.getHeadingEndless(), desiredAngle);

    SmartDashboard.putNumber("Error temp", error);
    SmartDashboard.putNumber("Output temp", output);

    chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 
    output, swerveSubsystem.getRotation2d());

    SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

    swerveSubsystem.setModuleStates(moduleStates);
    swerveSubsystem.updateSayac(); 

    System.out.println("Executing rotateThisMuch");
  }
    
  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stopModules();
    System.out.println("FINISHED rotateThisMuch");
  }

  @Override
  public boolean isFinished() {
    if(error < 3){return true;}
    else{return false;}
  }
}
