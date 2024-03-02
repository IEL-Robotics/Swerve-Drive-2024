package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class PneumaticSubsystem {

    private final DoubleSolenoid shootSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 1);

    public PneumaticSubsystem() {
        shootSolenoid.set(Value.kOff);
    }

    public void pistonOpen() {
        shootSolenoid.set(Value.kForward);
    }
    
    public void pistonClose() {
        shootSolenoid.set(Value.kReverse);
    }
}
