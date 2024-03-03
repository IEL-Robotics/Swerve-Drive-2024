package frc.robot.subsystems;

// import java.util.Collections;
// import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//users 10 photonconfig directory
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
    private PhotonCamera camera = new PhotonCamera("MyCamera");

    private AprilTagFieldLayout aprilTagFieldLayout;
     
    private Transform3d camera2Robot= new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(Math.toRadians(0.1), Math.toRadians(-42.88), Math.toRadians(-32.4)));
    private PhotonPipelineResult result;
    private PhotonTrackedTarget target;
    private Pose3d robotPose;
    double x = 0, y = 0, angle = 0, yaw = 0;
    double[] values = { x, y, angle };

    private PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
            PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camera, camera2Robot);

    public VisionSubsystem() {
        try {
            this.aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
        } catch (Exception e) {            
            e.printStackTrace();

        }
        targetTags();
    }

    public void targetTags() {
        camera.setPipelineIndex(0);
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return photonPoseEstimator.update();
    }

    public double[] getFieldPosition() {
        result = camera.getLatestResult();
        if (result.hasTargets()) {
            target = result.getBestTarget();
            if (target.getFiducialId() != -1) {
                robotPose = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(),
                        aprilTagFieldLayout.getTagPose(target.getFiducialId()).get(), camera2Robot);
                x = robotPose.getX();
                y = robotPose.getY();
                angle = Math.toDegrees(robotPose.getRotation().getAngle());

                SmartDashboard.putNumber("AprilTag ID:", target.getFiducialId());

                SmartDashboard.putNumber("X for Camera", x);
                SmartDashboard.putNumber("Y for Camera", y);
                SmartDashboard.putNumber("Angle for Camera", angle);
            }
        }

        values[0] = x - 0.35;
        values[1] = y - 0.25;
        values[2] = angle;

        SmartDashboard.putNumber("X of Robot", values[0]);
        SmartDashboard.putNumber("Y of Robot", values[1]);
        SmartDashboard.putNumber("Angle of Robot", values[2]);

        return values;
    }

    public int getTagID() {
        result = camera.getLatestResult();
        if (result.hasTargets()) {
            target = result.getBestTarget();
            return target.getFiducialId();
        }

        return -1;
    }

    public Pose2d getPoseOfRobot() {
        double[] positions = getFieldPosition();
        return new Pose2d(new Translation2d(positions[0], positions[1]), Rotation2d.fromDegrees(positions[2]));
    }

    public boolean gotDataIndeed() {
        if(values[0] != 0){return true;}
        else{return false;}
    }

}