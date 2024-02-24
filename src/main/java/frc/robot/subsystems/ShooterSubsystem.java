package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

    private CANSparkMax leftMotor, rightMotor;
    private RelativeEncoder leftEncoder, rightEncoder;

    public ShooterSubsystem() {
        leftMotor = new CANSparkMax(6, MotorType.kBrushless);
        rightMotor = new CANSparkMax(4, MotorType.kBrushless);

        // rightMotor.follow(leftMotor);

        leftMotor.setInverted(true);
        rightMotor.setInverted(false);

        leftMotor.setIdleMode(IdleMode.kCoast);
        rightMotor.setIdleMode(IdleMode.kCoast);

        leftEncoder = leftMotor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
        rightEncoder = rightMotor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
    }

    public void runMotors(double spd) {
        leftMotor.set(spd);
        rightMotor.set(spd);
    }
    
}