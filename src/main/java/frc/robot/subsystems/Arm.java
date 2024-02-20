package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants.ArmConstants;

public class Arm extends SubsystemBase {

    public VictorSPX leftMotor;
    public TalonSRX rightMotor;
    public PIDController leftPID, rightPID;

    public Arm() {
        leftMotor = new VictorSPX(ArmConstants.kLeftArmMotorId);
        rightMotor = new TalonSRX(ArmConstants.kRightArmMotorId);

        rightMotor.follow(leftMotor);

        leftMotor.setInverted(true);
        rightMotor.setInverted(false);

        leftMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
        rightMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);

        leftPID = new PIDController(0, 0, 0);
        rightPID = new PIDController(0, 0, 0);
    }

    public void armSet(double output) {
        leftMotor.set(VictorSPXControlMode.PercentOutput, output);
        rightMotor.set(TalonSRXControlMode.PercentOutput, output);
    }

    public double getLeftEncoderVal() {
        return leftMotor.getSelectedSensorPosition();
    }
    
    public double getRightEncoderVal() {
        return rightMotor.getSelectedSensorPosition();
    }

      @Override
  public void periodic() {
    SmartDashboard.putNumber("MagEncoder Left Value", leftMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("MagEncoder Right Value", rightMotor.getSelectedSensorPosition());
  }

}