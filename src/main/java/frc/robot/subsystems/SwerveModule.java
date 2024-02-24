package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
//rev
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

//import com.revrobotics.CANSparkMax.ControlType;
//wpi
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.SwerveSubsystemConstants;

public class SwerveModule extends SubsystemBase {
    
    private final CANSparkMax MOTOR_TURN;
    private final CANSparkMax MOTOR_DRIVE;

    private final RelativeEncoder ENCODER_TURN;
    private final RelativeEncoder ENCODER_DRIVE;

    private final SparkPIDController PID_VELOCITY;

    private final PIDController PID_TURNING;

    private final DutyCycleEncoder ENCODER_ABSOLUTE;
    private final double OFFSET_ABSOLUTEENCODER;

    private final String MODULE_NAME;

    private final SimpleMotorFeedforward FEEDFORWARD_VELOCITY;

    public SwerveModule(String _NAME, int ID_MOTOR_DRIVE, boolean REVERSE_MOTOR_DRIVE, int ID_MOTOR_TURN,
        boolean REVERSE_MOTOR_TURN, int ID_ENCODER_ABSOLUTE, boolean REVERSE_ENCODER_ABSOLUTE, double OFFSET_ENCODER_ABSOLUTE
    ) {

        this.MOTOR_DRIVE =new CANSparkMax(ID_MOTOR_DRIVE, MotorType.kBrushless);
        this.MOTOR_DRIVE.restoreFactoryDefaults();
        this.MOTOR_TURN = new CANSparkMax(ID_MOTOR_TURN, MotorType.kBrushless);
        this.MOTOR_TURN.restoreFactoryDefaults();

        MOTOR_DRIVE.setSmartCurrentLimit(30);
        MOTOR_TURN.setSmartCurrentLimit(20);

        MOTOR_DRIVE.setInverted(REVERSE_MOTOR_DRIVE);
        MOTOR_TURN.setInverted(REVERSE_MOTOR_TURN);

        MOTOR_DRIVE.setIdleMode(IdleMode.kBrake);
        MOTOR_TURN.setIdleMode(IdleMode.kBrake);

        MOTOR_DRIVE.setClosedLoopRampRate(0.0001);

        this.ENCODER_DRIVE = MOTOR_DRIVE.getEncoder(); //WHY NOT THIS -> getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42)
        this.ENCODER_TURN = MOTOR_TURN.getEncoder();

        ENCODER_DRIVE.setPositionConversionFactor(ModuleConstants.MODULE_DRIVE_ROTATIONS_TO_METERS);
        ENCODER_DRIVE.setVelocityConversionFactor(ModuleConstants.MODULE_DRIVE_RPM_TO_MPS);
        ENCODER_TURN.setPositionConversionFactor(ModuleConstants.MODULE_TURN_ROTATIONS_TO_RADIANS);
        ENCODER_TURN.setVelocityConversionFactor(ModuleConstants.TurningEncoderRPM2RadPerSec);

        PID_VELOCITY = MOTOR_DRIVE.getPIDController(); PID_VELOCITY.setP(0.0001); PID_VELOCITY.setI(0.00005); PID_VELOCITY.setD(0.0005);
        //PID_VELOCITY.setOutputRange(-AutonomousConstants.LIMIT_AUTOSPEED_DRIVE, AutonomousConstants.LIMIT_AUTOSPEED_DRIVE);
        
        FEEDFORWARD_VELOCITY = new SimpleMotorFeedforward(0.667, 2.44, 0.27);

        this.ENCODER_ABSOLUTE = new DutyCycleEncoder(ID_ENCODER_ABSOLUTE);
        this.ENCODER_ABSOLUTE.setDutyCycleRange(1.0/4096.0, 4095.0/4096.0);

        this.OFFSET_ABSOLUTEENCODER = OFFSET_ENCODER_ABSOLUTE;

        this.PID_TURNING = new PIDController(0.5,0.0, 0.0);
        PID_TURNING.enableContinuousInput(-Math.PI, Math.PI);

        MODULE_NAME = _NAME;

        prevAbsPos = getAbsoluteEncoder() - OFFSET_ABSOLUTEENCODER;
        currentAbsPos = getAbsoluteEncoder() - OFFSET_ABSOLUTEENCODER;
        currentRelativePosition = getAbsoluteEncoder() - OFFSET_ABSOLUTEENCODER;
        coefficient = 0;
    }

    public void moduleData2Dashboard(){
        //SmartDashboard.putNumber(String.format("%s AbsPos", MODULE_NAME), Math.toDegrees(getTurningPosition()));
        SmartDashboard.putNumber(MODULE_NAME + " Encoder",getDrivePosition());
    }

    public boolean checkIdle() {
        return (getDriveVelocity() != 0.0) && (getTurningVelocity() != 0.0);
    }

    public void resetTurnEncoders() {
        sayacUpdate();
        //ENCODER_TURN.setPosition(getAbsoluteEncoder());
    }
    
    public void resetDriveEncoders(){
        ENCODER_DRIVE.setPosition(0.0);
    }

    public void setDesiredState(SwerveModuleState state, boolean isOpenLoop) {
        state = SwerveModuleState.optimize(state, getModuleState().angle);
        setAngle(state);

        if(isOpenLoop){setPercentOutput(state);}
        else{setSpeed(state);}

        //SmartDashboard.putString(MODULE_NAME + " State", state.toString());
    }

    public void setAngle(SwerveModuleState state) {
        MOTOR_TURN.set(PID_TURNING.calculate(getTurningPosition(), state.angle.getRadians()));
    }

    public void setSpeed(SwerveModuleState state) {
        PID_VELOCITY.setReference(state.speedMetersPerSecond, ControlType.kVelocity,0,FEEDFORWARD_VELOCITY.calculate(state.speedMetersPerSecond));
    }

    public void setPercentOutput(SwerveModuleState state) {
        double percentOutput = state.speedMetersPerSecond/SwerveSubsystemConstants.LIMIT_HARD_SPEED_DRIVE;
        MOTOR_DRIVE.set(-percentOutput);
    }

    public double getDrivePosition(){
        return ENCODER_DRIVE.getPosition();
    }

    public double getTurningPosition(){ // Sayac Logic goes here
        //return ENCODER_TURN.getPosition();
        return sayacFinalVals();
    }

    public double getDriveVelocity(){
        return ENCODER_DRIVE.getVelocity();
    }

    public double getTurningVelocity(){
        return ENCODER_DRIVE.getVelocity();
    }

    public double getAbsoluteEncoder(){
        return ENCODER_ABSOLUTE.getAbsolutePosition() * 2 * Math.PI - Math.PI; // was + not -
    }

    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(getDrivePosition(),new Rotation2d(getTurningPosition()));
    }

    public void stop() {
        MOTOR_DRIVE.set(0);
        MOTOR_TURN.set(0);
    }

    /***************************************************************************************** */

    public double prevAbsPos, currentAbsPos;
    public int coefficient;
    public double currentRelativePosition;

    public int sayacCatchSwitch() {
      int diff = 0;
      if((prevAbsPos < -2.5) && (currentAbsPos > 2.5)){diff = -1;}
      if((prevAbsPos > 2.5) && (currentAbsPos < -2.5)){diff = +1;}
      return diff;
    }

    public double sayacFinalVals() {
      return 2*Math.PI*coefficient + currentAbsPos;
    }

    public void sayacUpdate() {
      prevAbsPos = currentAbsPos;
      currentAbsPos = getAbsoluteEncoder() - OFFSET_ABSOLUTEENCODER;  
    }
    
    public void sayacExecute() {
      sayacUpdate();
      SmartDashboard.putNumber(MODULE_NAME + " AbsEnc", currentAbsPos);
      coefficient += sayacCatchSwitch();
    }

}
