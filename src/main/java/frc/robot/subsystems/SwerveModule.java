package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.SwerveConstants.DriveConstants;
import frc.robot.Constants.SwerveConstants.ModuleConstants;
public class SwerveModule {

    private final int driveMotorId;
    public final CANSparkMax driveMotor,turningMotor;
    private final RelativeEncoder driveEnc,turningEnc;
    private final PIDController pidCont;
    
    public final DutyCycleEncoder dcEnc;
    private final boolean reversedAbsEnc;
    private final double absEncOffsetRad;

    private final String moduleName;

    public SwerveModule(int driveMotorId,int turningMotorId,boolean driveMotorReversed,boolean turningMotorReversed,
    int absoluteEncoderId, double absoluteEncoderOffset,boolean absoluteEncoderReversed, String moduleName){

        this.driveMotorId = driveMotorId;
        this.absEncOffsetRad = absoluteEncoderOffset;
        this.reversedAbsEnc = absoluteEncoderReversed;

        dcEnc = new DutyCycleEncoder(absoluteEncoderId);

        dcEnc.setDutyCycleRange(1.0/4096.0, 4095.0/4096.0);

        driveMotor = new CANSparkMax(driveMotorId, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
        turningMotor = new CANSparkMax(turningMotorId, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
        
        turningMotor.setIdleMode(com.revrobotics.CANSparkBase.IdleMode.kBrake);
        driveMotor.setIdleMode(com.revrobotics.CANSparkBase.IdleMode.kBrake);

        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);

        driveEnc = driveMotor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
        turningEnc = turningMotor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);

        driveEnc.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        driveEnc.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);

        //turningEnc.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
        //turningEnc.setPositionConversionFactor(0.2929938434314728);
        //turningEnc.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);
        //turningEnc.setVelocityConversionFactor(0.005018123109638691);    

        pidCont = new PIDController(0.5, 0.01, 0.00);
        pidCont.enableContinuousInput(-Math.PI, Math.PI);
        resetEnc();

        this.moduleName = moduleName;

        //*************************************************************************************** */  
        prevAbsPos = getAbsEncRad() - absEncOffsetRad;
        currentAbsPos = getAbsEncRad() - absEncOffsetRad;
        currentRelativePosition = getAbsEncRad() - absEncOffsetRad;
        coefficient = 0;
    }

    public double getDrivePosition(){
        return driveEnc.getPosition();
    }

    public double getTurningPosition(){
        //return turningEnc.getPosition();
        return sayacFinalVals();
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getTurningPosition()));  
    }

    public double getDriveVelocity(){
        return driveEnc.getVelocity();
    }

    public double getTurningVelocity(){
        return driveEnc.getVelocity();
    }

    public double getAbsEncRad(){
        return dcEnc.getAbsolutePosition() * 2 * Math.PI - Math.PI;
    }

    public void resetEnc(){
        driveEnc.setPosition(0);
        turningEnc.setPosition(getAbsEncRad() - absEncOffsetRad);
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(getDrivePosition(), new Rotation2d(getTurningPosition()));
    }

    public void setState(SwerveModuleState state){

        if (Math.abs(state.speedMetersPerSecond)<0.75){
            stop();
            return;
        }

        state=SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(state.speedMetersPerSecond/DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        turningMotor.set(pidCont.calculate(getTurningPosition(), state.angle.getRadians()));
    }

    public void stop(){
        driveMotor.set(0);
        turningMotor.set(0);
    }

    //*********************************************************************************************** */

    public double prevAbsPos, currentAbsPos;
    public int coefficient;
    public double currentRelativePosition;

    public int sayacCatchSwitch() {
      int diff = 0;
      if((prevAbsPos < -2.5) && (currentAbsPos > 2.5)){diff = -1;}
      if((prevAbsPos > 2.5) && (currentAbsPos < -2.5)){diff = +1;}
      return diff;
    }

    public void sayacDisplay() {
      SmartDashboard.putNumber(String.format("%s SayacVal", moduleName), currentAbsPos);
      SmartDashboard.putNumber(String.format("%s Coef", moduleName), coefficient);
      SmartDashboard.putNumber(String.format("%s DesiredVal", moduleName), 2*Math.PI*coefficient + currentAbsPos);
    }

    public double sayacFinalVals() {
      return 2*Math.PI*coefficient + currentAbsPos;
    }

    public void sayacUpdate() {
      prevAbsPos = currentAbsPos;
      currentAbsPos = getAbsEncRad() - absEncOffsetRad;
      sayacDisplay();    
    }
    
    public void sayacExecute() {
      sayacUpdate();
      coefficient += sayacCatchSwitch();
    }

}
