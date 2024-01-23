package frc.robot.commands.Autonumous;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveCertainDistance extends Command {
    double distance;
    private final SwerveSubsystem subsystem;
    private final DifferentialDrive diffDrive;
    public DriveCertainDistance(SwerveSubsystem subsystem,double distance) {
        this.distance=distance;
        this.subsystem=subsystem;
        this.subsystem.zeroHeading();
        this.subsystem.activateFollow();
        this.diffDrive=new DifferentialDrive(this.subsystem.frontLeft.driveMotor, 
                                            this.subsystem.frontRight.driveMotor);
        addRequirements(this.subsystem);
    }
    @Override
    public void end(boolean interrupted) {
        this.subsystem.stopModules();
    }
    
    @Override
    public void execute() {
        this.diffDrive.arcadeDrive(distance,0);
    }
    @Override
    public boolean isFinished() {
        if(Math.abs(distance - this.subsystem.frontLeft.driveEnc.getPosition()) > 0){
            return true;
          }
      
          return false;
    }
}

