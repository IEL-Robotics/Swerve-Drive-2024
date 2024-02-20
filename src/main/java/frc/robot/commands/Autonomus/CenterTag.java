// package frc.robot.commands.Autonomus;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.commands.Swerve.rotateThisMuch;
// import frc.robot.subsystems.SwerveSubsystem;
// import frc.robot.subsystems.Vision;

// public class CenterTag extends Command {

//     public SwerveSubsystem swerveSubsystem;
//     public Vision vision;
//     public rotateThisMuch m_RotateThisMuch;
//     boolean myInit = true;

//     public CenterTag(rotateThisMuch m_RotateThisMuch, Vision vision, SwerveSubsystem swerveSubsystem) {
//         this.swerveSubsystem = swerveSubsystem;
//         this.vision = vision;
//         this.m_RotateThisMuch = m_RotateThisMuch;
//         addRequirements();
//     }

//     @Override
//     public void initialize() {
//         //m_RotateThisMuch(swerveSubsystem, vision.getYaw());
//     }

//     @Override
//     public void execute() {
//         if(myInit){
//             //m_RotateThisMuch(swerveSubsystem, vision.getYaw());
//         }
//     }

//     @Override
//     public void end(boolean interrupted) {
//     }

//     @Override
//     public boolean isFinished() {
//         return false;
//     }
// }
