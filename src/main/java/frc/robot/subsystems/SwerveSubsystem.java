package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants.DriveConstants;

public class SwerveSubsystem extends SubsystemBase {

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

    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(
        DriveConstants.kDriveKinematics,
        new Rotation2d(0),
        new SwerveModulePosition[] { //Teleopta endeksler farkli oldugundan, burada da farkli sirayla mi koymaliyiz?
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        }, 
        new Pose2d(0.0, 0.0, new Rotation2d(0)) //AprilTag ile belirlemek? resetPosition() Metodu -> resetOdometry var zaten
    );

    public SwerveSubsystem() {
        new Thread(() -> {
            try {
                Thread.sleep(1000);//1000
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();
    }

    public void zeroHeading() {
        gyro.reset();
    }

    public double getHeading() {
        return -Math.IEEEremainder(gyro.getAngle(), 360);
        //return 0;
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public Pose2d getPose() {
        return odometer.getPoseMeters();
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
    public void periodic(){
        //odometer.update(getRotation2d(), frontLeft.getState(), frontRight.getState(), backLeft.getState(), backRight.getState());
       
        odometer.update(getRotation2d(), //Siralama konusundaki endise burada da gecerli
                        new SwerveModulePosition[] {
                            frontLeft.getPosition(),
                            frontRight.getPosition(), 
                            backLeft.getPosition(), 
                            backRight.getPosition()
                });

        SmartDashboard.putString("Konum", getPose().getTranslation().toString());
        
        SmartDashboard.putString("Aci", getRotation2d().toString());
        SmartDashboard.putNumber(" Heading Aci", getHeading());

    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setState(desiredStates[0]);
        frontRight.setState(desiredStates[1]);
        backLeft.setState(desiredStates[2]);
        backRight.setState(desiredStates[3]);
    }

    public void allValuesDisplay() {
      SmartDashboard.putNumber("LF Encoder", frontLeft.getAbsEncRad());
      SmartDashboard.putNumber("LB Encoder",  backLeft.getAbsEncRad());
      SmartDashboard.putNumber("RF Encoder",  frontRight.getAbsEncRad());
      SmartDashboard.putNumber("RB Encoder",  backRight.getAbsEncRad());

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
}
