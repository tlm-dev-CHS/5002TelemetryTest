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

    private final vision vision;

    private final PIDController rPidController;
    private final PIDController xPidController;
    private final PIDController yPidController;

    public autoRotate(CommandSwerveDrivetrain driveTrain, vision vision){

        this.vision = vision;

        this.driveTrain = driveTrain;
        currentPose = driveTrain.getState().Pose;
        requester = new SwerveRequest.FieldCentric();

        rPidController = new PIDController(0.15, 0.0, 0.0);
        xPidController = new PIDController(3, 0.0, 0.5);
        yPidController = new PIDController(3, 0.0, 0.5);


        xPidController.setTolerance(0.1);     
        yPidController.setTolerance(0.1); 
        rPidController.setTolerance(1);
    

    }

    public void moveToState(Enum goalPosition){
        
        PhotonTrackedTarget target = vision.getTracked();
        var goal = ((AutoAlignStates) goalPosition).getPose();

        int goalTag = (int) goal.get(0);
        Pose2d goalPose = (Pose2d) goal.get(1);

        double rOutput = rPidController.calculate(driveTrain.getState().Pose.getRotation().getDegrees(), 170);
        double xOutput = -xPidController.calculate(driveTrain.getState().Pose.getX(), 14.42);
        double yOutput = -yPidController.calculate(driveTrain.getState().Pose.getY(), 4.5);

        driveTrain.setControl(requester.withVelocityY(yOutput).withVelocityX(xOutput));
    
    }

    @Override

    public void periodic(){
        PhotonTrackedTarget target = vision.getTracked();
        
    }
}


