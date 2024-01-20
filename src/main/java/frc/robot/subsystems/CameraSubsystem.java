package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Constants;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.common.hardware.VisionLEDMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.targeting.PhotonPipelineResult;


public class CameraSubsystem extends SubsystemBase {
    private final PhotonCamera photonCamera;
    private final AprilTagFieldLayout aprilTagFieldLayout;
    private Pose3d estimatedRobotPose = new Pose3d();

    public CameraSubsystem() {
        this.photonCamera = new PhotonCamera("raven1288");
        //photonCamera.setLED(VisionLEDMode.kBlink);
        //if (!photonCamera.isConnected()) {
          //  throw new RuntimeException("Camera isn't connected!");
        //  }

        try {
            this.aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
        } catch (Exception e) {
            throw new RuntimeException("IOException while loading AprilTag field layout!");
        }
    }

    public Pose3d getLastEstimatedRobotPose() {
        return estimatedRobotPose;
    }

    public PhotonPipelineResult rawresult() {
        return photonCamera.getLatestResult();
    }

    @Override
    public void periodic() {
        PhotonPipelineResult result = photonCamera.getLatestResult();
        if (result.hasTargets()) {
            PhotonPoseEstimator poseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, new Transform3d()); // TODO: enable multitag PNP on CoProc (RasPi web interface)
            poseEstimator.setReferencePose(estimatedRobotPose);
            estimatedRobotPose = poseEstimator.update().get().estimatedPose;
        }
    }
}
