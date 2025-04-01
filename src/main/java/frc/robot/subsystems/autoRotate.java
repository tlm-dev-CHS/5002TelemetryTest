package frc.robot.subsystems;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AutoAlignStates;
import frc.robot.RobotContainer;
import frc.robot.subsystems.vision;

public class autoRotate extends SubsystemBase{

    private final Pose2d currentPose;

    private final CommandSwerveDrivetrain driveTrain;
    private final SwerveRequest.FieldCentric requester;

    private final PIDController rController;
    private final PIDController xPidController;
    private final PIDController yPidController;

    public autoRotate(CommandSwerveDrivetrain driveTrain){

        this.driveTrain = driveTrain;
        currentPose = driveTrain.getState().Pose;
        requester = new SwerveRequest.FieldCentric();

        rController = new PIDController(0.1, 0.0, 0.0);
        xPidController = new PIDController(0.1, 0.0, 0.0);
        yPidController = new PIDController(0.1, 0.0, 0.0);

        xPidController.setTolerance(0.1);     
        yPidController.setTolerance(0.1); 
        rController.setTolerance(1);

    }

    public void moveToState(Enum goalPosition, vision vision){
        
        PhotonTrackedTarget target = vision.getTracked();
        var goal = ((AutoAlignStates) goalPosition).getPose();

        var j = Constants.AutoAlignStates.BLUE_INTAKE.getPose();
        Pose2d pose = (Pose2d) j.get(1);
        int k = (int) j.get(0);

        
    
    }
}


