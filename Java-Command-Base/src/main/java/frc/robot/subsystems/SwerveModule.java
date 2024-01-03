package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.SwerveConstants.ModuleConstants;
import frc.robot.Constants.SwerveConstants.DriveConstants;
public class SwerveModule {

    private final int driveMotorId;
    private final CANSparkMax driveMotor,turningMotor;
    private final RelativeEncoder driveEnc,turningEnc;
    private final PIDController pidCont;
    private final AnalogInput absEnc;

    private final DutyCycleEncoder dcEnc;
    private final boolean reversedAbsEnc;
    private final double absEncOffsetRad;

    public SwerveModule(int driveMotorId,int turningMotorId,boolean driveMotorReversed,boolean turningMotorReversed,
    int absoluteEncoderId, double absoluteEncoderOffset,boolean absoluteEncoderReversed){

        this.driveMotorId = driveMotorId;
        this.absEncOffsetRad = absoluteEncoderOffset;
        this.reversedAbsEnc = absoluteEncoderReversed;
        absEnc = new AnalogInput(absoluteEncoderId);

        dcEnc = new DutyCycleEncoder(absoluteEncoderId);

        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        turningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);

        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);

        driveEnc = driveMotor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
        turningEnc = turningMotor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);

        driveEnc.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        driveEnc.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        turningEnc.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
        turningEnc.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

        pidCont = new PIDController(ModuleConstants.kPTurning, 0, 0);
        pidCont.enableContinuousInput(-Math.PI, Math.PI);
        this.resetEnc();

    }

    public double getDrivePosition(){
        return this.driveEnc.getPosition();
    }

    public double getTurningPosition(){
        return this.turningEnc.getPosition();
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(this.getDrivePosition(),new Rotation2d(this.getTurningPosition()));  
    }

    public double getDriveVelocity(){
        return this.driveEnc.getVelocity();
    }

    public double getTurningVelocity(){
        return this.driveEnc.getVelocity();
    }

    public double getAbsEncRad(){
        // turningMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.fromId(0)); //Belki?

        // double ang=this.absEnc.getVoltage()/RobotController.getVoltage5V();
        // ang*=Math.PI*2;
        // ang-=this.absEncOffsetRad;
        // if(this.reversedAbsEnc) return -ang;
        // else return ang;

        return dcEnc.getAbsolutePosition() * 2 * Math.PI - Math.PI;


    }

    public void resetEnc(){
        driveEnc.setPosition(0);
        turningEnc.setPosition(this.getAbsEncRad());
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(this.getDrivePosition(),new Rotation2d(this.getTurningPosition()));
    }

    public void setState(SwerveModuleState state){

        if (Math.abs(state.speedMetersPerSecond)<0.25){
            stop();
            return;
        }

        SmartDashboard.putNumber(String.format("Encoder %d", driveMotorId), dcEnc.getAbsolutePosition());
        state=SwerveModuleState.optimize(state, getState().angle);
        this.driveMotor.set(state.speedMetersPerSecond/DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        this.turningMotor.set(this.pidCont.calculate(this.getTurningPosition(),state.angle.getRadians()));
    }

    public void stop(){
        this.driveMotor.set(0);
        this.turningMotor.set(0);
    }

}
