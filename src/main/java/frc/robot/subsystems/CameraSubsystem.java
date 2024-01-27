package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import frc.robot.Constants;

/**
 * Subsystem for Position Estimation based on AprilTags
 */
public class CameraSubsystem extends SubsystemBase {
    private final PhotonCamera photonCamera;
    private final AprilTagFieldLayout aprilTagFieldLayout;
    private final PhotonPoseEstimator poseEstimator;
    private Pose3d estimatedRobotPose = new Pose3d();

    /**
     * Initialises the Camera subsystem with the camera named in ModuleConstants
     */
    public CameraSubsystem() {
        photonCamera = new PhotonCamera(Constants.ModuleConstants.CAMERA_NAME);

        try {
            this.aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
            poseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, photonCamera, new Transform3d());
        } catch (Exception e) {
            throw new RuntimeException("IOException while loading AprilTag field layout!");
        }
    }

    /**
     * Gets the current best guess position of the robot.
     * @param debug Prints debug messages.
     * @return Pose3d of estimated robot position.
     */
    public Pose3d getLastEstimatedRobotPose(boolean debug) {
        if (debug) {
            System.out.println("Got estimated position of X: " + estimatedRobotPose.getX() + ", Y: " + estimatedRobotPose.getY() + " Z: " + estimatedRobotPose.getZ());
        }
        return estimatedRobotPose;
    }

    @Override
    public void periodic() {
        PhotonPipelineResult result = photonCamera.getLatestResult();
        if (result.hasTargets()) {
            poseEstimator.setReferencePose(estimatedRobotPose);
            // gets the estimated pose
            Optional<EstimatedRobotPose> estimatedPoseOptional = poseEstimator.update();
            if (estimatedPoseOptional.isPresent()) {
                estimatedRobotPose = estimatedPoseOptional.get().estimatedPose;
            }
        }
    }
}
