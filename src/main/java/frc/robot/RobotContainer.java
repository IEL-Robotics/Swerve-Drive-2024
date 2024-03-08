package frc.robot;

import java.lang.module.ModuleDescriptor.Requires;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Debug;
import frc.robot.commands.Delay;
import frc.robot.commands.Arm.LowerArm;
import frc.robot.commands.Arm.PresetArm;
import frc.robot.commands.Arm.RaiseArm;
import frc.robot.commands.Intake.IntakeOut;
import frc.robot.commands.Intake.IntakeStart;
import frc.robot.commands.Intake.IntakeStop;
import frc.robot.commands.Intake.Intakeing;
import frc.robot.commands.Pneumatic.PistonClose;
import frc.robot.commands.Pneumatic.PistonOpen;
import frc.robot.commands.Shooter.Shoot;
import frc.robot.commands.Shooter.ShooterChain;
import frc.robot.commands.Shooter.ShooterStart;
import frc.robot.commands.Shooter.ShooterStop;
import frc.robot.commands.Swerve.ResetGyro;
import frc.robot.commands.Swerve.RotateThisMuch;
import frc.robot.commands.Swerve.RotateToSpecificAngle;
import frc.robot.commands.Swerve.StopModules;
import frc.robot.commands.Swerve.SwerveDrive;
import frc.robot.commands.Vision.Aim;
import frc.robot.commands.Vision.CenterSpeaker;
import frc.robot.commands.Vision.ExperimentalSetStart;
import frc.robot.commands.Vision.PathAndShootCmd;
import frc.robot.commands.Vision.VisionSetStart;
//import frc.robot.commands.Vision.VisionSetStart;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PneumaticSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class RobotContainer {

  public final PS4Controller JOYSTICK_DRIVER = new PS4Controller(0);
  public final PS4Controller JOYSTICK_COPILOT = new PS4Controller(1);

  public final SwerveSubsystem SUBSYSTEM_SWERVEDRIVE = new SwerveSubsystem();
  public final ArmSubsystem SUBSYSTEM_ARM = new ArmSubsystem();
  public final ShooterSubsystem SUBSYSTEM_SHOOTER = new ShooterSubsystem();
  public final PneumaticSubsystem SUBSYSTEM_PNEUMATIC = new PneumaticSubsystem();
  public final IntakeSubsystem SUBSYSTEM_INTAKE = new IntakeSubsystem();
  public final VisionSubsystem SUBSYSTEM_VISION = new VisionSubsystem();

  public final ResetGyro RESET_GYRO = new ResetGyro(SUBSYSTEM_SWERVEDRIVE);
  public final RotateThisMuch ROTATE_THIS_MUCH = new RotateThisMuch(SUBSYSTEM_SWERVEDRIVE, 0);
  public final RotateToSpecificAngle ROTATE_TO_SPECIFIC_ANGLE = new RotateToSpecificAngle(SUBSYSTEM_SWERVEDRIVE, 0);

  public final LowerArm LOWER_ARM = new LowerArm(SUBSYSTEM_ARM);
  public final RaiseArm RAISE_ARM = new RaiseArm(SUBSYSTEM_ARM);

  public final PistonOpen PISTON_OPEN = new PistonOpen(SUBSYSTEM_PNEUMATIC);
  public final PistonClose PISTON_CLOSE = new PistonClose(SUBSYSTEM_PNEUMATIC);

  public final Shoot SHOOT = new Shoot(SUBSYSTEM_SHOOTER);
  public final ShooterStop SHOOTER_STOP = new ShooterStop(SUBSYSTEM_SHOOTER);
  public final ShooterChain SHOOTER_CHAIN = new ShooterChain(new ShooterStart(SUBSYSTEM_SHOOTER, 1), new ShooterStop(SUBSYSTEM_SHOOTER), new PistonClose(SUBSYSTEM_PNEUMATIC), new PistonOpen(SUBSYSTEM_PNEUMATIC), 1);
  public final ShooterChain SHOOTER_CHAIN_WEAK = new ShooterChain(new ShooterStart(SUBSYSTEM_SHOOTER, 0.5), new ShooterStop(SUBSYSTEM_SHOOTER), new PistonClose(SUBSYSTEM_PNEUMATIC), new PistonOpen(SUBSYSTEM_PNEUMATIC), 0.35);

  public final Intakeing INTAKE_IN = new Intakeing(SUBSYSTEM_INTAKE);
  public final IntakeOut INTAKE_OUT = new IntakeOut(SUBSYSTEM_INTAKE);

  //public final VisionSetStart VISION_SET_START = new VisionSetStart(SUBSYSTEM_VISION, SUBSYSTEM_SWERVEDRIVE);
  public final CenterSpeaker CENTER_SPEAKER = new CenterSpeaker(SUBSYSTEM_SWERVEDRIVE, SUBSYSTEM_VISION);

  public final Aim AIM = new Aim(SUBSYSTEM_ARM, SUBSYSTEM_VISION, JOYSTICK_DRIVER);

  // Trigger DRIVER_START= new Trigger( () -> JOYSTICK_DRIVER.getRawButton(7));
  // Trigger DRIVER_BACK = new Trigger( () -> JOYSTICK_DRIVER.getRawButton(8));

  public RobotContainer() {

    //SmartDashboard.putData(SUBSYSTEM_SWERVEDRIVE);

    SUBSYSTEM_SWERVEDRIVE.setDefaultCommand(
        new SwerveDrive(
            SUBSYSTEM_SWERVEDRIVE,
            () -> -
            JOYSTICK_DRIVER.getRawAxis(1),
            () -> -JOYSTICK_DRIVER.getRawAxis(0),
            () -> JOYSTICK_DRIVER.getRawAxis(2),
            () -> (JOYSTICK_DRIVER.getRawAxis(3) > 0)
        )
    );

    NamedCommands.registerCommand("Debug", new Debug());
    NamedCommands.registerCommand("StopModules", new StopModules(SUBSYSTEM_SWERVEDRIVE));
    NamedCommands.registerCommand("ResetModulePosition", SUBSYSTEM_SWERVEDRIVE.zeroModuleAngles());
    NamedCommands.registerCommand("VisionStart", new VisionSetStart(SUBSYSTEM_VISION, SUBSYSTEM_SWERVEDRIVE));
    NamedCommands.registerCommand("CenterAim1", CENTER_SPEAKER.withTimeout(2));
    NamedCommands.registerCommand("CenterAim2", AIM.withTimeout(2));
    NamedCommands.registerCommand("ArmToIntake", new PresetArm(SUBSYSTEM_ARM, -1235).withTimeout(2));
    NamedCommands.registerCommand("ArmRaiseLittleBit", new PresetArm(SUBSYSTEM_ARM, -1100).withTimeout(2));
    NamedCommands.registerCommand("IntakeStart", new IntakeStart(SUBSYSTEM_INTAKE));
    NamedCommands.registerCommand("IntakeStop", new IntakeStop(SUBSYSTEM_INTAKE));
    NamedCommands.registerCommand("ShooterChain", SHOOTER_CHAIN);
    NamedCommands.registerCommand("ShooterStart", new ShooterStart(SUBSYSTEM_SHOOTER, 1));
    NamedCommands.registerCommand("ShooterStop", new ShooterStop(SUBSYSTEM_SHOOTER));
    NamedCommands.registerCommand("PistonOpen", PISTON_OPEN);
    NamedCommands.registerCommand("PistonClose", PISTON_CLOSE);


    configureBindings();
  }

  private void configureBindings() {

    //new JoystickButton(JOYSTICK_DRIVER, 1).onTrue(new PresetArm(SUBSYSTEM_ARM, 995)); //Square - Intake
    new JoystickButton(JOYSTICK_DRIVER, 1).onTrue(new PresetArm(SUBSYSTEM_ARM, -1235)); 
    new JoystickButton(JOYSTICK_DRIVER, 2).onTrue(new PresetArm(SUBSYSTEM_ARM, -470)); // Cross - Amplifier // R1 //umutcum preseti değiştirdim saygılar
    new JoystickButton(JOYSTICK_DRIVER, 10).onTrue(RESET_GYRO); // Options
    new JoystickButton(JOYSTICK_DRIVER, 4).onTrue(SHOOTER_CHAIN_WEAK);
    //new JoystickButton(JOYSTICK_DRIVER, 3).onTrue(CENTER_SPEAKER);
    new JoystickButton(JOYSTICK_DRIVER, 3).onTrue(new SequentialCommandGroup(CENTER_SPEAKER, AIM)); 
    //new JoystickButton(JOYSTICK_DRIVER, 3).onTrue(new SequentialCommandGroup(new PathAndShootCmd(SUBSYSTEM_SWERVEDRIVE, new VisionSetStart(SUBSYSTEM_VISION, SUBSYSTEM_SWERVEDRIVE), "Turnike"), new PresetArm(SUBSYSTEM_ARM, 995), SHOOTER_CHAIN));
    //new JoystickButton(JOYSTICK_DRIVER, 3).onTrue(AIM);
    //new JoystickButton(JOYSTICK_DRIVER, 3).onTrue(new PathAndShootCmd(SUBSYSTEM_SWERVEDRIVE, new VisionSetStart(SUBSYSTEM_VISION, SUBSYSTEM_SWERVEDRIVE), "Turnike"));
    new JoystickButton(JOYSTICK_DRIVER, 8).onTrue(SHOOTER_CHAIN);
    new JoystickButton(JOYSTICK_DRIVER, 5).whileTrue(LOWER_ARM);
    new JoystickButton(JOYSTICK_DRIVER, 6).whileTrue(RAISE_ARM);

    new JoystickButton(JOYSTICK_COPILOT, 1).onTrue(PISTON_OPEN);
    new JoystickButton(JOYSTICK_COPILOT, 3).onTrue(PISTON_CLOSE);
    new JoystickButton(JOYSTICK_COPILOT, 7).whileTrue(INTAKE_OUT); // Faulty 
    new JoystickButton(JOYSTICK_COPILOT, 8).whileTrue(INTAKE_IN); // Faulty

  }

  public Command getAutonomousCommand() {
    return new PathPlannerAuto("Red 1"); // ?

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

}
