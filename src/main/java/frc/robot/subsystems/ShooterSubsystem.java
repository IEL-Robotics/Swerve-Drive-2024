package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

    private CANSparkMax leftMotor, rightMotor;
    private RelativeEncoder leftEncoder, rightEncoder;
    private PowerDistribution pdp;

    public ShooterSubsystem() {
        leftMotor = new CANSparkMax(6, MotorType.kBrushless);
        rightMotor = new CANSparkMax(4, MotorType.kBrushless);
        pdp = new PowerDistribution(2, ModuleType.kRev);

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

    @Override
    public void periodic() {
        SmartDashboard.putNumber("LeftRPM", leftEncoder.getVelocity());
        SmartDashboard.putNumber("RightRPM", rightEncoder.getVelocity());
        if(leftEncoder.getVelocity() > 3500){
            //System.out.println("Current: "+ pdp.getCurrent(0));
            //System.out.println(leftEncoder.getVelocity()+ " & " + rightEncoder.getVelocity());
        }
    }

    public double[] debug() {
        SmartDashboard.putNumber("LeftRPM", leftEncoder.getVelocity());
        SmartDashboard.putNumber("RightRPM", rightEncoder.getVelocity());
        return new double[]{leftEncoder.getVelocity(), rightEncoder.getVelocity()};
    }
    
}
