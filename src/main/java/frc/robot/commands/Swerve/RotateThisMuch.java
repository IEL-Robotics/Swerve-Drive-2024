package frc.robot.commands.Swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.KinematicsConstants;
import frc.robot.Constants.SwerveSubsystemConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class RotateThisMuch extends Command {

  public SwerveSubsystem swerveSubsystem;
  public PIDController pidController = new PIDController(0.25, 0.25, 0.005);
  ChassisSpeeds chassisSpeeds;

  double output, initialAngle, desiredAngle, currentAngle;
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

  public void reInit() {
    initialAngle = swerveSubsystem.getRobotHeadingEndless();
    pidController.setSetpoint(desiredAngle + initialAngle);
    pidController.setTolerance(2.5, 10);
    myInit = false;
  }

  @Override
  public void execute() {
    if (myInit) {
      reInit();
    }

    currentAngle = swerveSubsystem.getRobotHeadingEndless();
    output = MathUtil.clamp(pidController.calculate(currentAngle, desiredAngle + initialAngle),
        -SwerveSubsystemConstants.LIMIT_SOFT_SPEED_TURN, -SwerveSubsystemConstants.LIMIT_SOFT_SPEED_TURN);

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
    return pidController.atSetpoint();
  }

}
