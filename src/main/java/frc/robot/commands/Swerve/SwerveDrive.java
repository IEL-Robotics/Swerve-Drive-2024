package frc.robot.commands.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;

//import edu.wpi.first.math.filter.SlewRateLimiter;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveSubsystemConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveDrive extends Command {

    private final SwerveSubsystem subsystem;
    private final DoubleSupplier SUPPLIER_xSpeed;
    private final DoubleSupplier SUPPLIER_ySpeed;
    private final DoubleSupplier SUPPLIER_zSpeed;

    private final SlewRateLimiter xLimiter, yLimiter, zLimiter;

    public SwerveDrive(SwerveSubsystem m_subsystem, DoubleSupplier xSpeed, DoubleSupplier ySpeed,
        DoubleSupplier zSpeed, BooleanSupplier Field_Oriented){

        this.subsystem = m_subsystem;
        this.SUPPLIER_xSpeed = xSpeed;
        this.SUPPLIER_ySpeed = ySpeed;
        this.SUPPLIER_zSpeed = zSpeed;
        
        addRequirements(subsystem);

        this.xLimiter = new SlewRateLimiter(3);
        this.yLimiter = new SlewRateLimiter(3);
        this.zLimiter = new SlewRateLimiter(3);
    }

    @Override
    public void execute() {
        /*
        //Get joystick input from double suppliers
        double xSpeed = SUPPLIER_xSpeed.getAsDouble() * Constants.SwerveSubsystemConstants.LIMIT_SOFT_SPEED_DRIVE * 0.2 * (Math.abs(SUPPLIER_xSpeed.getAsDouble()) > 0.1 ? 1.0 : 0.0);
        double ySpeed = SUPPLIER_ySpeed.getAsDouble() * Constants.SwerveSubsystemConstants.LIMIT_SOFT_SPEED_DRIVE * 0.2 * (Math.abs(SUPPLIER_ySpeed.getAsDouble()) > 0.1 ? 1.0 : 0.0);
        double rotSpeed = SUPPLIER_zSpeed.getAsDouble()* Constants.SwerveSubsystemConstants.LIMIT_SOFT_SPEED_TURN * 0.2;
        */

        // double joystickX = SUPPLIER_xSpeed.getAsDouble() * (Math.abs(SUPPLIER_xSpeed.getAsDouble()) > 0.05 ? 1.0 : 0.0);
        // double joystickY = SUPPLIER_ySpeed.getAsDouble() * (Math.abs(SUPPLIER_ySpeed.getAsDouble()) > 0.05 ? 1.0 : 0.0); //grab speeds and apply deadband
        // double joystickZ = SUPPLIER_zSpeed.getAsDouble() * (Math.abs(SUPPLIER_zSpeed.getAsDouble()) > 0.05 ? 1.0 : 0.0);


        // double xSpeed   = (Math.pow(joystickX, 2) * (joystickX<0 ? -1 : 1) /1.0) *   SwerveSubsystemConstants.LIMIT_SOFT_SPEED_DRIVE * (RobotContainer.DRIVER_LT() ? 0.3 : 1);      // * 0.2;
        // double ySpeed   = (Math.pow(joystickY, 2) * (joystickY<0 ? -1 : 1) /1.0) *   SwerveSubsystemConstants.LIMIT_SOFT_SPEED_DRIVE * (RobotContainer.DRIVER_LT() ? 0.3 : 1);      // * 0.2; //Determine new velocity
        // double rotSpeed = (Math.pow(joystickZ, 2) * (joystickZ<0 ? -1 : 1) /1.0)*0.8*SwerveSubsystemConstants.LIMIT_SOFT_SPEED_TURN  * (RobotContainer.DRIVER_LT() ? 0.3 : 1); //* 0.2 to make SLOW */
        double xSpeed = SUPPLIER_xSpeed.getAsDouble();
        double ySpeed = SUPPLIER_ySpeed.getAsDouble();
        double rotSpeed = SUPPLIER_zSpeed.getAsDouble();

        xSpeed = Math.abs(xSpeed) > 0.1 ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > 0.1 ? ySpeed : 0.0;
        rotSpeed = Math.abs(rotSpeed) > 0.12 ? rotSpeed : 0.0;

        xSpeed = xLimiter.calculate(xSpeed) * SwerveSubsystemConstants.LIMIT_SOFT_SPEED_DRIVE;
        ySpeed = yLimiter.calculate(ySpeed) * SwerveSubsystemConstants.LIMIT_SOFT_SPEED_DRIVE;
        rotSpeed = zLimiter.calculate(rotSpeed) * SwerveSubsystemConstants.LIMIT_SOFT_SPEED_TURN;

        //subsystem.updateSayac();
        // SmartDashboard.putNumber("xSpeed", xSpeed);
        // SmartDashboard.putNumber("ySpeed", ySpeed);
        // SmartDashboard.putNumber("rotSpeed", rotSpeed);
        ChassisSpeeds chassisSpeed = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotSpeed, subsystem.getRotation2d());
        //ChassisSpeeds chassisSpeed = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, subsystem.getRotation2d());
        subsystem.setChassisSpeed(chassisSpeed,true);
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.setChassisSpeed(ChassisSpeeds.fromFieldRelativeSpeeds(0,0,0,subsystem.getRotation2d()), true);
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
}
