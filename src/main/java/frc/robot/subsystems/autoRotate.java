package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class autoRotate extends SubsystemBase{
    private final PhotonCamera camera;
    private final PIDController rPidController;
    private final SwerveRequest.RobotCentric requester;
    private double setPoint;
    private int targetID;

    public autoRotate(int targetAID){
        camera = new PhotonCamera("photonvision");
        rPidController = new PIDController(.5, 0, 0);
        rPidController.setTolerance(0.1);
        requester = new RobotCentric();
        targetID = targetAID;
        setPoint = 0.0;
    }

    public void rotate(){
        PhotonPipelineResult result = camera.getLatestResult();
        boolean hasTargets = result.hasTargets();

        if (hasTargets){
            PhotonTrackedTarget target = result.getBestTarget();
            if (target.getFiducialId() == targetID){
                Transform3d bestCameraToTarget = target.getBestCameraToTarget();
        
            }

        }
        //boolean hasTargets = camera.
        //if (result != none)
    }
}

