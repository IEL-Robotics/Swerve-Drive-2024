package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.geometry.Pose2d;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import frc.robot.Constants.SwerveConstants.AutoConstants;
import frc.robot.Constants.SwerveConstants.DriveConstants;
import frc.robot.subsystems.SwerveModule;

public class SwerveSubsystem extends SubsystemBase {
    public boolean isFollow=false;
    public final SwerveModule frontLeft = new SwerveModule(
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort,
            DriveConstants.kFrontLeftDriveEncoderReversed,
            DriveConstants.kFrontLeftTurningEncoderReversed,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed,
            "Sol On");

    public final SwerveModule frontRight = new SwerveModule(
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightDriveEncoderReversed,
            DriveConstants.kFrontRightTurningEncoderReversed,
            DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
            DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontRightDriveAbsoluteEncoderReversed,
            "Sag On");

    public final SwerveModule backLeft = new SwerveModule(
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftTurningMotorPort,
            DriveConstants.kBackLeftDriveEncoderReversed,
            DriveConstants.kBackLeftTurningEncoderReversed,
            DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
            DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackLeftDriveAbsoluteEncoderReversed,
            "Sol Arka");

    public final SwerveModule backRight = new SwerveModule(
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightTurningMotorPort,
            DriveConstants.kBackRightDriveEncoderReversed,
            DriveConstants.kBackRightTurningEncoderReversed,
            DriveConstants.kBackRightDriveAbsoluteEncoderPort,
            DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackRightDriveAbsoluteEncoderReversed,
            "Sag Arka");
    private final AHRS gyro = new AHRS(SPI.Port.kMXP);
    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(DriveConstants.kDriveKinematics,
            new Rotation2d(0), new SwerveModulePosition[] {
                    frontLeft.getPosition(),
                    frontRight.getPosition(),
                    backLeft.getPosition(),
                    backRight.getPosition()
            }, new Pose2d(0.0, 0.0, new Rotation2d(0)));

    public SwerveSubsystem() {
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();
        AutoBuilder.configureHolonomic(
            this::getPose, // Robot pose supplier
            this::setPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                    AutoConstants.kMaxSpeedMetersPerSecond, // Max module speed, in m/s
                    Math.hypot(DriveConstants.kTrackWidth / 2, DriveConstants.kWheelBase / 2), // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            () -> {
              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this
    );
    }

    public void zeroHeading() {
        gyro.reset();
    }

    public double getHeading() {
      // SmartDashboard.putNumber("Gyro Abs", gyro.getAngle());
      // SmartDashboard.putNumber("Gyro Remain", Math.IEEEremainder(gyro.getAngle(), 360));
        return -Math.IEEEremainder(gyro.getAngle(), 360);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }
    public void setPose(Pose2d pose) {
        odometer.resetPosition(getRotation2d(), getModulePositions(), pose);
    }

    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(new Rotation2d(), new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
        },pose);
    }
    @Override
    public void periodic(){}
    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setState(desiredStates[1]);
        frontRight.setState(desiredStates[0]);
        backLeft.setState(desiredStates[3]);
        backRight.setState(desiredStates[2]);
    }

    public void allValuesDisplay() {
      SmartDashboard.putNumber("LF Encoder", frontLeft.getAbsEncRad());
      SmartDashboard.putNumber("LB Encoder",  backLeft.getAbsEncRad());
      SmartDashboard.putNumber("RF Encoder",  frontRight.getAbsEncRad());
      SmartDashboard.putNumber("RB Encoder",  backRight.getAbsEncRad());

      //SmartDashboard.putNumber("RF Moduler", frontRight.getTurningPosition() % (Math.PI));

      SmartDashboard.putNumber("LF getTurn", frontLeft.getTurningPosition());
      SmartDashboard.putNumber("LB getTurn",  backLeft.getTurningPosition());
      SmartDashboard.putNumber("RF getTurn",  frontRight.getTurningPosition());
      SmartDashboard.putNumber("RB getTurn",  backRight.getTurningPosition());
    }

    public void updateSayac() {
      frontRight.sayacExecute();
      frontLeft.sayacExecute();
      backLeft.sayacExecute();
      backRight.sayacExecute();
    }
    public double[] getDriveEncoderPosition(){
        return new double[] {
            frontRight.getDrivePosition(),

            frontLeft.getDrivePosition(),

            backRight.getDrivePosition(),

            backLeft.getDrivePosition()
        };
    }
    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions={
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        };
        return positions;
    }
    public ChassisSpeeds getRobotRelativeSpeeds() {
        SwerveModuleState[] states = new SwerveModuleState[]{
            frontLeft.getState(),
            frontRight.getState(),
            backLeft.getState(),
            backRight.getState()
        };
        return DriveConstants.kDriveKinematics.toChassisSpeeds(states);
    }
    public void driveRobotRelative(ChassisSpeeds speeds) {
        setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds));
    }
}