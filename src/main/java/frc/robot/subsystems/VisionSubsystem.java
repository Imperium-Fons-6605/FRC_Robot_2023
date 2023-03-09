package frc.robot.subsystems;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Util.Constants.VisionConstants;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
public class VisionSubsystem {
    private final PhotonCamera m_camera;
    private PhotonPoseEstimator m_photonPoseEstimator;

    public VisionSubsystem(){
        m_camera = new PhotonCamera(VisionConstants.kCameraName);
        try {
            AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
            m_photonPoseEstimator =
                    new PhotonPoseEstimator(
                            fieldLayout, PoseStrategy.MULTI_TAG_PNP, m_camera, VisionConstants.kTransformRobotToCam);
            m_photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        } catch (IOException e) {
            DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
            m_photonPoseEstimator = null;
        }
    }

    /**
     * @param estimatedRobotPose The current best guess at robot pose
     * @return an EstimatedRobotPose with an estimated pose, the timestamp, and targets used to create
     *     the estimate
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        if (m_photonPoseEstimator == null) {
            // The field layout failed to load, so we cannot estimate poses.
            return Optional.empty();
        }
        m_photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return m_photonPoseEstimator.update();
    }
    

}
