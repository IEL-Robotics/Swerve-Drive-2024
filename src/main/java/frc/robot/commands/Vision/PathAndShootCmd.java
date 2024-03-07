package frc.robot.commands.Vision;

import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class PathAndShootCmd extends SequentialCommandGroup {

    public SwerveSubsystem swerveSubsystem;
    public VisionSetStart visionSetStart;

    public PathAndShootCmd(SwerveSubsystem swerveSubsystem, VisionSetStart visionSetStart, String pathName) {
        addCommands(visionSetStart, swerveSubsystem.followPathCommand(pathName));
    }

}
