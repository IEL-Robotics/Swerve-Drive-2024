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
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//users 10 photonconfig directory
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
    private PhotonCamera camera = new PhotonCamera("MyCamera");

    private AprilTagFieldLayout aprilTagFieldLayout;

    private Transform3d camera2Robot = new Transform3d(new Translation3d(0.0, 0.0, 0.0), new Rotation3d(0, 0, 0)); // madeup
                                                                                                                   // //
                                                                                                                   // value
    private PhotonPipelineResult result;
    private PhotonTrackedTarget target;
    private Pose3d robotPose;
    double x = 0, y = 0, angle = 0, yaw = 0;
    double[] values = { x, y, angle };

    private PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
            PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camera, camera2Robot);

    public Vision() {
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
                x = robotPose.getTranslation().getX();
                y = robotPose.getTranslation().getY();
                angle = robotPose.getRotation().getAngle();

                SmartDashboard.putNumber("AprilTag ID:", target.getFiducialId());
            }
        }

        values[0] = x;
        values[1] = y;
        values[2] = angle;

        SmartDashboard.putNumber("X in Field", values[0]);
        SmartDashboard.putNumber("Y in Field", values[1]);
        SmartDashboard.putNumber("Angle in Field", values[2]);

        return values;
    }

    public int getTagID() {
        result = camera.getLatestResult();
        if (result.hasTargets()) {
            target = result.getBestTarget();

            // SmartDashboard.putNumber("DynamicX",
            //         PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(),
            //                 aprilTagFieldLayout.getTagPose(target.getFiducialId()).get(), camera2Robot).getX());
            // SmartDashboard.putNumber("DynamicY",
            //         PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(),
            //                 aprilTagFieldLayout.getTagPose(target.getFiducialId()).get(), camera2Robot).getY());
            // SmartDashboard.putNumber("GBCTT X", target.getBestCameraToTarget().getTranslation().getX());
            // SmartDashboard.putNumber("GBCTT Y", target.getBestCameraToTarget().getTranslation().getY());

            return target.getFiducialId();
        }

        return -1;
    }

    public double getYaw() {
        result = camera.getLatestResult();
        if (result.hasTargets()) {
            target = result.getBestTarget();

            yaw = target.getYaw();
            SmartDashboard.putNumber("GetYawToTag", yaw);

            return yaw;
        }
        return yaw;
    }

}