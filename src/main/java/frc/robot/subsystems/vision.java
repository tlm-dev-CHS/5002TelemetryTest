package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.PhotonUtils;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class vision extends SubsystemBase{
    private Pose2d prevPose2d;
        private final PhotonCamera camera; 
        private final AprilTagFieldLayout aprilTagFieldLayout;
        private final Transform3d robotToCam;
        private final PhotonPoseEstimator photonPoseEstimator;
        private final VisionSystemSim visionSim;
        private final CommandSwerveDrivetrain drivetrain;
    
        public vision(CommandSwerveDrivetrain drivetrain){ 
            
            prevPose2d = drivetrain.getState().Pose;
            visionSim = new VisionSystemSim("main");
            camera = new PhotonCamera(Constants.OperatorConstants.cameraName);
            aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
            visionSim.addAprilTags(aprilTagFieldLayout);
            robotToCam = new Transform3d(new Translation3d(0.3302, 0.1016, 0.3048), new Rotation3d(0,0,0));
            photonPoseEstimator =  new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCam);
    
            this.drivetrain = drivetrain;
    
        }
    
        public PhotonPipelineResult getResult(){
            return camera.getLatestResult();
        }
    
        public PhotonTrackedTarget getTracked(){
            PhotonPipelineResult result = getResult();
            if (result != null){
                return result.getBestTarget();
            }
            else{
                return null;
            }
        }
    
        public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
            PhotonPipelineResult result = getResult();
    
            if (prevEstimatedRobotPose != null){
                photonPoseEstimator.setReferencePose(prevEstimatedRobotPose); 
            }
            if (result != null){
                return photonPoseEstimator.update(result);
            }
            else{
                return Optional.empty();
            }
        }
    
        @Override
        public void periodic(){
            PhotonPipelineResult result = getResult();
            if (result != null && result.hasTargets()){
                
                Optional<EstimatedRobotPose> estimation = getEstimatedGlobalPose(prevPose2d);
            
                if (estimation.isPresent()){
                    Pose2d pose = estimation.get().estimatedPose.toPose2d();
                    prevPose2d = pose;
                System.out.println(pose);
                var timestamp = result.getTimestampSeconds();
                System.out.println(timestamp);
                drivetrain.addVisionMeasurement(pose, timestamp);
            }

        }
    }
}
