package frc.robot;

import java.lang.module.ModuleDescriptor.Requires;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Debug;
import frc.robot.commands.Delay;
import frc.robot.commands.Arm.LowerArm;
import frc.robot.commands.Arm.RaiseArm;
import frc.robot.commands.Shooter.Shoot;
import frc.robot.commands.Swerve.ResetGyro;
import frc.robot.commands.Swerve.RotateThisMuch;
import frc.robot.commands.Swerve.RotateToSpecificAngle;
import frc.robot.commands.Swerve.StopModules;
import frc.robot.commands.Swerve.SwerveDrive;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {

  private final static PS4Controller JOYSTICK_DRIVER = new PS4Controller(0);

  public final SwerveSubsystem SUBSYSTEM_SWERVEDRIVE = new SwerveSubsystem();
  public final ArmSubsystem SUBSYSTEM_ARM = new ArmSubsystem();
  public final ShooterSubsystem SUBSYSTEM_SHOOTER = new ShooterSubsystem();

  public final ResetGyro RESET_GYRO = new ResetGyro(SUBSYSTEM_SWERVEDRIVE);
  public final RotateThisMuch ROTATE_THIS_MUCH = new RotateThisMuch(SUBSYSTEM_SWERVEDRIVE, 0);
  public final RotateToSpecificAngle ROTATE_TO_SPECIFIC_ANGLE = new RotateToSpecificAngle(SUBSYSTEM_SWERVEDRIVE, 0);
  
  public final LowerArm LOWER_ARM = new LowerArm(SUBSYSTEM_ARM);
  public final RaiseArm RAISE_ARM = new RaiseArm(SUBSYSTEM_ARM);

  public final Shoot SHOOT = new Shoot(SUBSYSTEM_SHOOTER);

  private final Rotation2d rotation = new Rotation2d(0);
  private final Pose2d pose = new Pose2d(2.5,5.5, rotation);

  //Trigger DRIVER_START= new Trigger( () -> JOYSTICK_DRIVER.getRawButton(7));
  //Trigger DRIVER_BACK = new Trigger( () -> JOYSTICK_DRIVER.getRawButton(8));

  public RobotContainer() {

    SmartDashboard.putData(SUBSYSTEM_SWERVEDRIVE);

    SUBSYSTEM_SWERVEDRIVE.setDefaultCommand(
      new SwerveDrive(
        SUBSYSTEM_SWERVEDRIVE, 
        () -> JOYSTICK_DRIVER.getRawAxis(1), 
        () -> JOYSTICK_DRIVER.getRawAxis(0), 
        () -> JOYSTICK_DRIVER.getRawAxis(2), 
        () -> true
      )
    );

    NamedCommands.registerCommand("Debug", new Debug());
    NamedCommands.registerCommand("StopModules", new StopModules(SUBSYSTEM_SWERVEDRIVE));
    NamedCommands.registerCommand("ResetModulePosition", SUBSYSTEM_SWERVEDRIVE.zeroModuleAngles());

    configureBindings();
  }

  private void configureBindings() {
    new JoystickButton(JOYSTICK_DRIVER, 2).onTrue(RESET_GYRO);
    //DRIVER_START.whileTrue(SUBSYSTEM_SWERVEDRIVE.zeroModuleAngles());
    //DRIVER_BACK.onTrue(SUBSYSTEM_SWERVEDRIVE.lockDrive());
       // Circle
       new JoystickButton(JOYSTICK_DRIVER, 3).onTrue(ROTATE_THIS_MUCH);
       // Square
       new JoystickButton(JOYSTICK_DRIVER, 1).onTrue(ROTATE_THIS_MUCH);
       // Triangle
       new JoystickButton(JOYSTICK_DRIVER, 4).onTrue(ROTATE_TO_SPECIFIC_ANGLE);
       //R1
       new JoystickButton(JOYSTICK_DRIVER, 5).whileTrue(RAISE_ARM);
       //L1
       new JoystickButton(JOYSTICK_DRIVER, 6).whileTrue(LOWER_ARM);

       new JoystickButton(JOYSTICK_DRIVER, 7).whileTrue(SHOOT);

  }

  public Command getAutonomousCommand() {
        return new PathPlannerAuto("New Auto"); //?

        // PathPlannerPath path1 = PathPlannerPath.fromPathFile("Example Path");
        // PathPlannerPath path2 = PathPlannerPath.fromPathFile("Example Path Reversed");

        // return new SequentialCommandGroup(
        //   AutoBuilder.followPath(path1),
        //   new StopModules(SUBSYSTEM_SWERVEDRIVE),
        //   new WaitCommand(5),
        //   AutoBuilder.followPath(path2),
        //   new StopModules(SUBSYSTEM_SWERVEDRIVE)
        // );
  } 
}
