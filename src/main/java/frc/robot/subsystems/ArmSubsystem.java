package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {

    public TalonSRX leftMotor,rightMotor;
    public PIDController leftPID, rightPID;

    public double armEncoderVariable = 0;
    public double armEncoderOffset = 0;

    public ArmSubsystem() {
        leftMotor = new TalonSRX(ArmConstants.ID_LEFT_ARM);
        rightMotor = new TalonSRX(ArmConstants.ID_RIGHT_ARM);

        rightMotor.follow(leftMotor);

        leftMotor.setInverted(true);
        rightMotor.setInverted(false);

        leftMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
        rightMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);

        leftPID = new PIDController(0, 0, 0);
        rightPID = new PIDController(0, 0, 0);

        armEncoderOffset =leftMotor.getSelectedSensorPosition();
    }

    public void armSet(double output) {
        leftMotor.set(TalonSRXControlMode.PercentOutput, output);
        rightMotor.set(TalonSRXControlMode.PercentOutput, output);
    }

    public double getLeftEncoderVal() {
        return leftMotor.getSelectedSensorPosition();
    }
    
    public double getRightEncoderVal() {
        //return rightMotor.getSelectedSensorPosition();
        return rightMotor.getSelectedSensorPosition();
        //return rightMotor.getSelectedSensorPosition() - armEncoderOffset + 500; //500 is the start config for arm
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("MagEncoder Left Value", getLeftEncoderVal());
        //SmartDashboard.putNumber("MagEncoder Right Value", rightMotor.getSelectedSensorPosition());
    }

}