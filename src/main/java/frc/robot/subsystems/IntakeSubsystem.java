package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

    private TalonSRX intakeMotor;

    public IntakeSubsystem() {
        intakeMotor = new TalonSRX(13);
        intakeMotor.setInverted(false);
    }

    public void runMotors() {
        intakeMotor.set(TalonSRXControlMode.PercentOutput, 1);
    }

    public void runMotorsInverted() {
        intakeMotor.set(TalonSRXControlMode.PercentOutput, -1);
    }

        public void stopMotors() {
        intakeMotor.set(TalonSRXControlMode.PercentOutput, 0);
    }
    
}
