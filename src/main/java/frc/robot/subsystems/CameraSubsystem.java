package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Constants;
import org.photonvision.PhotonCamera;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;


public class CameraSubsystem extends SubsystemBase {
    private final PhotonCamera photonCamera;
    private final AprilTagFieldLayout aprilTagFieldLayout;

    public CameraSubsystem() {
        this.photonCamera = new PhotonCamera(Constants.ModuleConstants.CAMERA_NAME);
        if (!photonCamera.isConnected()) {
            throw new RuntimeException("Camera isn't connected!");
        }

        try {
            this.aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
        } catch (Exception e) {
            throw new RuntimeException("IOException while loading AprilTag field layout!");
        }
    }

    @Override
    public void periodic() {
        PhotonPipelineResult result = photonCamera.getLatestResult();
        if (result.hasTargets()) {
            PhotonTrackedTarget target = result.getBestTarget();
            PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(), aprilTagFieldLayout.getTagPose(target.getFiducialId()).get(), new Transform3d());
            
        }   
    }
}
