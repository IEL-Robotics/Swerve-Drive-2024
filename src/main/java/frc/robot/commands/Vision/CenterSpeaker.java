package frc.robot.commands.Vision;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.KinematicsConstants;
import frc.robot.Constants.SwerveSubsystemConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class CenterSpeaker extends Command {

  public SwerveSubsystem swerveSubsystem;
  public VisionSubsystem visionSubsystem;
  public PIDController pidController = new PIDController(0.15, 0.025, 0.005);
  ChassisSpeeds chassisSpeeds;

  double output, desiredAngle, currentAngle, realDesiredAngle;
  int kConstant;
  boolean myInit = true;

  public CenterSpeaker(SwerveSubsystem swerveSubsystem, VisionSubsystem visionSubsystem) {
    this.swerveSubsystem = swerveSubsystem;
    this.visionSubsystem = visionSubsystem;
    addRequirements(this.swerveSubsystem);
  }

  @Override
  public void initialize() {
    kConstant = (int) swerveSubsystem.getRobotHeadingEndless() / 360;
    realDesiredAngle = (desiredAngle) + (360 * kConstant);
  }

  public double centerAngle() {
    double[] positons = visionSubsystem.getFieldPosition();
    return visionSubsystem.centerAngle(positons[0], positons[1]);
  }

  public void reInit() {
    kConstant = (int) swerveSubsystem.getRobotHeadingEndless() / 360;
    realDesiredAngle = (centerAngle()) + (360 * kConstant);
    pidController.setSetpoint(realDesiredAngle);
    pidController.setTolerance(2, 10);
    pidController.setIZone(10);
    myInit = false;
  }

  @Override
  public void execute() {
    if (myInit) {
      reInit();
    }

    System.out.println("CENTERSPEAKER RUNNING");

    currentAngle = swerveSubsystem.getRobotHeadingEndless();
    output = MathUtil.clamp(pidController.calculate(currentAngle, realDesiredAngle),
        -1, 1);

    chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, -output, swerveSubsystem.getRotation2d());

    // SwerveModuleState[] moduleStates = KinematicsConstants.KINEMATICS_DRIVE_CHASSIS.toSwerveModuleStates(chassisSpeeds);

    // swerveSubsystem.setModuleStates(moduleStates, true); // ?
    swerveSubsystem.setChassisSpeed(chassisSpeeds, true);
    swerveSubsystem.updateSayac();
  }

  @Override
  public void end(boolean interrupted) {
    myInit = true;
    System.out.println("ENDED***********************");
    swerveSubsystem.stopModules();
  }

  @Override
  public boolean isFinished() {
    return pidController.atSetpoint();
  }
}

// package frc.robot.commands.Vision;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.math.kinematics.SwerveModuleState;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants.KinematicsConstants;
// import frc.robot.Constants.SwerveSubsystemConstants;
// import frc.robot.subsystems.SwerveSubsystem;
// import frc.robot.subsystems.VisionSubsystem;

// public class CenterSpeaker extends Command {

//   public SwerveSubsystem swerveSubsystem;
//   public VisionSubsystem visionSubsystem;
//   public PIDController pidController = new PIDController(0.25, 0.25, 0.005);
//   ChassisSpeeds chassisSpeeds;

//   double output;

//   boolean myInit = true;

//   public CenterSpeaker(SwerveSubsystem swerveSubsystem, VisionSubsystem visionSubsystem) {
//     this.swerveSubsystem = swerveSubsystem;
//     this.visionSubsystem = visionSubsystem;
//     addRequirements(this.swerveSubsystem);
//   }

//   @Override
//   public void initialize() {
//   }

//   public void reInit() {
//     pidController.setSetpoint(0.0);
//     pidController.setTolerance(0.02, 10);
//     pidController.setIZone(0.3);
//     myInit = false;
//   }

//   @Override
//   public void execute() {
//     if (myInit) {reInit();}

//     System.out.println("CENTERSPEAKER RUNNING");
//     System.out.println(visionSubsystem.centerTag());

//     output = MathUtil.clamp(pidController.calculate(visionSubsystem.centerTag()),
//         -SwerveSubsystemConstants.LIMIT_SOFT_SPEED_TURN * 0.5, SwerveSubsystemConstants.LIMIT_SOFT_SPEED_TURN * 0.5);

//     chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, -output, swerveSubsystem.getRotation2d());

//     SwerveModuleState[] moduleStates = KinematicsConstants.KINEMATICS_DRIVE_CHASSIS.toSwerveModuleStates(chassisSpeeds);

//     swerveSubsystem.setModuleStates(moduleStates, false); // ?
//     swerveSubsystem.updateSayac();
//   }

//   @Override
//   public void end(boolean interrupted) {
//     myInit = true;
//     System.out.println("ENDED***********************");
//     visionSubsystem.prevCenterTag = 0;
//     swerveSubsystem.stopModules();
//   }

//   @Override
//   public boolean isFinished() {
//     return pidController.atSetpoint();
//   }
// }


// //NOW IM STARTING HAHAHAHAHHAHAHAHAHAHHAHAHHA