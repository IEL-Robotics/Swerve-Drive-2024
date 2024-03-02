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

    //private Transform3d camera2Robot = new Transform3d(new Translation3d(0.332, -0.25, 0.32), new Rotation3d(0, -0.66185, -0.5235)); // madeup
                                                                                                                   // // pitch -45, yaw -35.5, 33.041
     
    private Transform3d camera2Robot= new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(Math.toRadians(0.1), Math.toRadians(-42.88), Math.toRadians(-32.4)));                                                                                                            
    private Transform3d camera2Robot2 =  new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0,0, 0));                                                                                                         // value
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

                SmartDashboard.putNumber("X in FieldN", x);
                SmartDashboard.putNumber("Y in FieldN", y);
                SmartDashboard.putNumber("Angle in FieldN", angle);
            }
        }

        values[0] = x;
        values[1] = y;
        values[2] = angle;

        return values;
    }

        public double[] getFieldPosition2() {
        result = camera.getLatestResult();
        if (result.hasTargets()) {
            target = result.getBestTarget();
            if (target.getFiducialId() != -1) {
                robotPose = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(),
                        aprilTagFieldLayout.getTagPose(target.getFiducialId()).get(), camera2Robot2);
                x = robotPose.getX();
                y = robotPose.getY();
                angle = Math.toDegrees(robotPose.getRotation().getAngle());

                // SmartDashboard.putNumber("X Alternative", x + 0.355);
                // SmartDashboard.putNumber("Y Alternative", y + 0.25);
                // SmartDashboard.putNumber("Angle Alternative", angle + 13);
                SmartDashboard.putNumber("X AlternativeN", x);
                SmartDashboard.putNumber("Y AlternativeN", y);
                SmartDashboard.putNumber("Angle AlternativeN", angle);
            }
        }

        values[0] = x;
        values[1] = y;
        values[2] = angle;

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

    public Pose2d getPoseOfRobot() {
        double[] positions = getFieldPosition();
        return new Pose2d(new Translation2d(positions[0], positions[1]), Rotation2d.fromDegrees(positions[2]));
    }

    public boolean gotDataIndeed() {
        if(values[0] != 0){return true;}
        else{return false;}
    }

    public void visionCompare() {
        result = camera.getLatestResult();
        if (result.hasTargets()) {
            target = result.getBestTarget();
            if (target.getFiducialId() != -1) {
                Pose3d robotPose1 = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(),
                        aprilTagFieldLayout.getTagPose(target.getFiducialId()).get(), camera2Robot2);

                Pose3d robotPoseRaw = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(),
                        aprilTagFieldLayout.getTagPose(target.getFiducialId()).get(), camera2Robot);

                SmartDashboard.putNumber("AprilTag ID:", target.getFiducialId());

                SmartDashboard.putNumber("X Raw", robotPoseRaw.getX());
                SmartDashboard.putNumber("Y Raw", robotPoseRaw.getY());
                SmartDashboard.putNumber("Angle Raw", Math.toDegrees(robotPoseRaw.getRotation().getAngle()));

                SmartDashboard.putNumber("X 1", robotPose1.getX());
                SmartDashboard.putNumber("Y 1", robotPose1.getY());
                SmartDashboard.putNumber("Angle 1", Math.toDegrees(robotPose1.getRotation().getAngle()));

                SmartDashboard.putNumber("CamToTarget angle", target.getBestCameraToTarget().getRotation().toRotation2d().getDegrees());
                SmartDashboard.putString("CamToTarget", target.getBestCameraToTarget().toString());
                SmartDashboard.putNumber("TargetPitch", target.getPitch());
            }
        }
    }

}