// package frc.robot.commands.Autonomus;

// import frc.robot.Constants.SwerveConstants.AutoConstants;
// import frc.robot.Constants.SwerveConstants.DriveConstants;
// import frc.robot.subsystems.SwerveSubsystem;
// import frc.robot.subsystems.Vision;

// import java.util.ArrayList;
// import java.util.List;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.trajectory.Trajectory;
// import edu.wpi.first.math.trajectory.TrajectoryConfig;
// import edu.wpi.first.math.trajectory.TrajectoryGenerator;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

// public class TrajectoryDrive1 extends SequentialCommandGroup {
//   SwerveSubsystem swerveSubsystem;
//   Vision vision;
//   double[] positions;

//   public TrajectoryDrive1(SwerveSubsystem swerveSubsystem, Vision vision, double[] endX, double[] endY, double endTheta, boolean firstCommand) {
//     if(firstCommand){
//       this.vision = vision;
//       setOdometryVision();
//     }
//     else{
//       Pose2d pose = swerveSubsystem.getPose();
//       positions[0] = pose.getX();
//       positions[1] = pose.getY();      
//       positions[2] = pose.getRotation().getDegrees();
//     }

//     ArrayList<Translation2d> wayPoints=new ArrayList<Translation2d>();
//     for(int i=0;i<endX.length-1;i++){
//       assert wayPoints.add(new Translation2d(endX[i],endY[i]));
//     }

//     TrajectoryConfig trajectoryConfig = new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
//         AutoConstants.kMaxAccelerationMetersPerSecondSquared).setKinematics(DriveConstants.kDriveKinematics);
//     Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
//         new Pose2d(positions[0] * 3.28084, positions[1] * 3.28084, Rotation2d.fromDegrees(positions[2])), // old -> new Rotation2d(radians)
//         wayPoints,
//         new Pose2d(endX[endX.length-1]* 3.28084, endY[endY.length-1] * 3.28084, Rotation2d.fromDegrees(endTheta)),
//         trajectoryConfig);

//     PIDController xController = new PIDController(0.5, 0.01, 0.0);
//     PIDController yController = new PIDController(0.5, 0.01, 0.0);
//     ProfiledPIDController thetaController = new ProfiledPIDController(
//         0.75, 0.15, 0.0035, AutoConstants.kThetaControllerConstraints);
//     thetaController.enableContinuousInput(-Math.PI, Math.PI);

//     SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
//         trajectory, swerveSubsystem::getPose, DriveConstants.kDriveKinematics, xController, yController,
//         thetaController,
//         swerveSubsystem::setModuleStates, swerveSubsystem);

//     addCommands(new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory.getInitialPose())),
//         swerveControllerCommand,
//         new InstantCommand(() -> swerveSubsystem.stopModules())); // comment out?
//   }

//   public void setOdometryVision() {
//     do {
//       positions = vision.getFieldPosition();
//     } while (positions[0] != 0 && positions[1] != 0);
//   }
// }
