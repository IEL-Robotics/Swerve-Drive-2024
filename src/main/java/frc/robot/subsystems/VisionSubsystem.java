package frc.robot.subsystems;

// import java.util.Collections;
// import java.util.List;
import java.util.Optional;

import javax.print.attribute.standard.MediaSize.NA;

import org.opencv.core.Mat;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.fasterxml.jackson.annotation.ObjectIdGenerators.None;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//users 10 photonconfig directory
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
    private PhotonCamera camera = new PhotonCamera("BRIO_4K_Stream_Edition");

    private AprilTagFieldLayout aprilTagFieldLayout;
     
    //private Transform3d camera2Robot= new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0));
    private Transform3d camera2Robot= new Transform3d(new Translation3d(-0.0, -0.0, 0.0), new Rotation3d(Math.toRadians(1.08), Math.toRadians(-35.2), Math.toRadians(159.06)));
    //private Transform3d camera2Robot= new Transform3d(new Translation3d(-0.35, -0.27, 0.0), new Rotation3d(Math.toRadians(1.08), Math.toRadians(-35.2), Math.toRadians(159.06)));
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

    public void displayCameraPos() {
        result = camera.getLatestResult();
        if(result.hasTargets()){
            target = result.getBestTarget();
            if(target.getFiducialId() != -1){
                    robotPose = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(),
                        aprilTagFieldLayout.getTagPose(target.getFiducialId()).get(), new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(Math.toRadians(0), Math.toRadians(0), Math.toRadians(0))));
            }
        }
    }

    public double[] getFieldPosition() {
        result = camera.getLatestResult();
        if (result.hasTargets()) {
            target = result.getBestTarget();
            if (target.getFiducialId() != -1) {
                displayCameraPos();
                robotPose = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(),
                        aprilTagFieldLayout.getTagPose(target.getFiducialId()).get(), camera2Robot);
                
                x = robotPose.getX();
                y = robotPose.getY();
                //angle = Math.toDegrees(robotPose.getRotation().getAngle());

                SmartDashboard.putString("RobotPose", robotPose.toString());

                angle = getAngleViaId((target.getBestCameraToTarget().getRotation().getAngle() * 180.0/Math.PI) - 24 + 43, target.getFiducialId());
                SmartDashboard.putNumber("AprilTag ID:", target.getFiducialId());

                SmartDashboard.putNumber("X for Camera", x);
                SmartDashboard.putNumber("Y for Camera", y);
                SmartDashboard.putNumber("Angle for Calculation", angle);

                double xFactor = Math.abs(-0.35 * Math.cos(Math.toRadians(angle)) + -0.27 * Math.sin(Math.toRadians(angle)));
                double yFactor = Math.abs(-0.35 * Math.sin(Math.toRadians(angle)) + -0.27 * Math.cos(Math.toRadians(angle)));
                x -= DriverStation.getAlliance().get() == DriverStation.Alliance.Red ? xFactor : -xFactor;
                y -= DriverStation.getAlliance().get() == DriverStation.Alliance.Red ? yFactor : -yFactor;
            }
        }

        values[0] = x;
        values[1] = y;
        values[2] = angle;

        SmartDashboard.putNumber("X of Robot", values[0]);
        SmartDashboard.putNumber("Y of Robot", values[1]);
        SmartDashboard.putNumber("Angle of Robot", values[2]);

        return values;
    }

    public double getAngleViaId(double yaw ,int id){
        if(id == 3 || id == 4 || id == 7 || id == 8){
            return yaw - 180;
        }
        else{
            return 0;
        }
    }

    public double prevCenterTag = 0;

    public double centerTag(){
        result = camera.getLatestResult();
        if (result.hasTargets()) {
            target = result.getBestTarget();
            if (target.getFiducialId() != -1) {
                prevCenterTag = target.getBestCameraToTarget().getY();
                return prevCenterTag;
            }
        }
        return prevCenterTag;
    }

    public double centerAngle(double xIn, double yIn) {
        double xDistance = DriverStation.getAlliance().get() == DriverStation.Alliance.Blue ? xIn - (-0.04) : 16.58 - xIn;
        double yDistance = yIn - 5.55;
        SmartDashboard.putNumber("ATAN2", Math.toDegrees(Math.atan2(yDistance, xDistance)));
        return -Math.toDegrees(Math.atan2(yDistance, xDistance)); 
    }

    public double getDistanceForBlue(double xIn, double yIn){
        double distance = Math.sqrt(Math.pow(-0.04 - xIn, 2) + Math.pow(5.55 - yIn, 2)); 
        return distance;
    }

    public double getDistanceForRed(double xIn, double yIn){
        double distance = Math.sqrt(Math.pow(16.58 - xIn, 2) + Math.pow(5.55 - yIn, 2)); 
        SmartDashboard.putNumber("Distance from Reg", distance);
        return distance;
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