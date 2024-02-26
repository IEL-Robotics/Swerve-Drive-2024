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
import frc.robot.commands.Pneumatic.PistonClose;
import frc.robot.commands.Pneumatic.PistonOpen;
import frc.robot.commands.Shooter.Shoot;
import frc.robot.commands.Shooter.ShooterStart;
import frc.robot.commands.Shooter.ShooterStop;
import frc.robot.commands.Swerve.ResetGyro;
import frc.robot.commands.Swerve.RotateThisMuch;
import frc.robot.commands.Swerve.RotateToSpecificAngle;
import frc.robot.commands.Swerve.StopModules;
import frc.robot.commands.Swerve.SwerveDrive;
import frc.robot.commands.Vision.ExperimentalSetStart;
import frc.robot.commands.Vision.VisionSetStart;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.PneumaticSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class RobotContainer {

  private final static PS4Controller JOYSTICK_DRIVER = new PS4Controller(0);
  private final static PS4Controller JOYSTICK_COPILOT = new PS4Controller(0);

  public final SwerveSubsystem SUBSYSTEM_SWERVEDRIVE = new SwerveSubsystem();
  public final ArmSubsystem SUBSYSTEM_ARM = new ArmSubsystem();
  public final ShooterSubsystem SUBSYSTEM_SHOOTER = new ShooterSubsystem();
  public final PneumaticSubsystem SUBSYSTEM_PNEUMATIC = new PneumaticSubsystem();
  public final VisionSubsystem SUBSYSTEM_VISION = new VisionSubsystem();

  public final ResetGyro RESET_GYRO = new ResetGyro(SUBSYSTEM_SWERVEDRIVE);
  public final RotateThisMuch ROTATE_THIS_MUCH = new RotateThisMuch(SUBSYSTEM_SWERVEDRIVE, 0);
  public final RotateToSpecificAngle ROTATE_TO_SPECIFIC_ANGLE = new RotateToSpecificAngle(SUBSYSTEM_SWERVEDRIVE, 0);

  public final LowerArm LOWER_ARM = new LowerArm(SUBSYSTEM_ARM);
  public final RaiseArm RAISE_ARM = new RaiseArm(SUBSYSTEM_ARM);

  public final Shoot SHOOT = new Shoot(SUBSYSTEM_SHOOTER);
  public final ShooterStart SHOOTER_START = new ShooterStart(SUBSYSTEM_SHOOTER);
  public final ShooterStop SHOOTER_STOP = new ShooterStop(SUBSYSTEM_SHOOTER);
  public final Command SHOOTER_CHAIN = shooterChainCommand();

  public final PistonOpen PISTON_OPEN = new PistonOpen(SUBSYSTEM_PNEUMATIC);
  public final PistonClose PISTON_CLOSE = new PistonClose(SUBSYSTEM_PNEUMATIC);

  // Trigger DRIVER_START= new Trigger( () -> JOYSTICK_DRIVER.getRawButton(7));
  // Trigger DRIVER_BACK = new Trigger( () -> JOYSTICK_DRIVER.getRawButton(8));

  public RobotContainer() {

    SmartDashboard.putData(SUBSYSTEM_SWERVEDRIVE);

    SUBSYSTEM_SWERVEDRIVE.setDefaultCommand(
        new SwerveDrive(
            SUBSYSTEM_SWERVEDRIVE,
            () -> JOYSTICK_DRIVER.getRawAxis(1),
            () -> JOYSTICK_DRIVER.getRawAxis(0),
            () -> JOYSTICK_DRIVER.getRawAxis(2),
            () -> true));

    NamedCommands.registerCommand("Debug", new Debug());
    NamedCommands.registerCommand("StopModules", new StopModules(SUBSYSTEM_SWERVEDRIVE));
    NamedCommands.registerCommand("ResetModulePosition", SUBSYSTEM_SWERVEDRIVE.zeroModuleAngles());
    NamedCommands.registerCommand("VisionStart", new VisionSetStart(SUBSYSTEM_VISION, SUBSYSTEM_SWERVEDRIVE));
    NamedCommands.registerCommand("ExperimentalStart", new ExperimentalSetStart(SUBSYSTEM_SWERVEDRIVE));

    configureBindings();
  }

  private void configureBindings() {
    new JoystickButton(JOYSTICK_DRIVER, 2).onTrue(RESET_GYRO);
    // DRIVER_START.whileTrue(SUBSYSTEM_SWERVEDRIVE.zeroModuleAngles());
    // DRIVER_BACK.onTrue(SUBSYSTEM_SWERVEDRIVE.lockDrive());

    new JoystickButton(JOYSTICK_DRIVER, 5).whileTrue(RAISE_ARM);
    new JoystickButton(JOYSTICK_DRIVER, 6).whileTrue(LOWER_ARM);

    new JoystickButton(JOYSTICK_DRIVER, 7).whileTrue(SHOOT);

    new JoystickButton(JOYSTICK_COPILOT, 3).onTrue(ROTATE_THIS_MUCH);
    new JoystickButton(JOYSTICK_COPILOT, 1).onTrue(ROTATE_THIS_MUCH);
    new JoystickButton(JOYSTICK_COPILOT, 4).onTrue(ROTATE_TO_SPECIFIC_ANGLE);

  }

  public Command getAutonomousCommand() {
    return new PathPlannerAuto("New Auto"); // ?

    // PathPlannerPath path1 = PathPlannerPath.fromPathFile("Example Path");
    // PathPlannerPath path2 = PathPlannerPath.fromPathFile("Example Path
    // Reversed");

    // return new SequentialCommandGroup(
    // AutoBuilder.followPath(path1),
    // new StopModules(SUBSYSTEM_SWERVEDRIVE),
    // new WaitCommand(5),
    // AutoBuilder.followPath(path2),
    // new StopModules(SUBSYSTEM_SWERVEDRIVE)
    // );
  }

  public Command shooterChainCommand() {
    return new SequentialCommandGroup(
        SHOOTER_START, new WaitCommand(0.5),
        PISTON_CLOSE, new WaitCommand(0.3),
        PISTON_OPEN, SHOOTER_STOP);
  }
}
