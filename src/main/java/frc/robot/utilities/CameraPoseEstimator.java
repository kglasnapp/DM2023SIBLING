package frc.robot.utilities;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Transform3d;

/**
 * This class was created as a way to filter out images where
 * the ambiguity was too high.
 * It is used instead of the super class PhotonPoseEstimator
 */
public class CameraPoseEstimator extends PhotonPoseEstimator {
    
    /**
     * Any apriltag with an ambiguity higher than this number will be ignored.`
     */
    public final static double MAX_AMBIGUITY = 0.2;

    public CameraPoseEstimator(AprilTagFieldLayout fieldTags, PoseStrategy strategy, PhotonCamera camera,
            Transform3d robotToCamera) {
        super(fieldTags, strategy, camera, robotToCamera);
    }

    @Override
    public Optional<EstimatedRobotPose> update(PhotonPipelineResult cameraResult) {
        List<PhotonTrackedTarget> targets = cameraResult.getTargets();
        for (int i=targets.size()-1;i>0;i--) {
            PhotonTrackedTarget target = targets.get(i);
            if (target.getPoseAmbiguity() < MAX_AMBIGUITY) {
                targets.remove(i);
            }
        }
        return super.update(cameraResult);
    }
}
