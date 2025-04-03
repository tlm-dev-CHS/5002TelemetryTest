package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AutoAlignStates;
import frc.robot.RobotContainer;
import frc.robot.subsystems.vision;

public class autoRotate extends SubsystemBase{

    public Map currentPose;
    
        private final CommandSwerveDrivetrain driveTrain;
        private final SwerveRequest.FieldCentric requester;
    
        private final vision vision;
    
        private final PIDController rPidController;
        private final PIDController xPidController;
        private final PIDController yPidController;
    
        public autoRotate(CommandSwerveDrivetrain driveTrain, vision vision){
    
            this.vision = vision;
    
            this.driveTrain = driveTrain;
            currentPose = new HashMap<>();
            requester = new SwerveRequest.FieldCentric();
    
            rPidController = new PIDController(0.15, 0.0, 0.0);
            xPidController = new PIDController(3, 0.0, 0.5);
            yPidController = new PIDController(3, 0.0, 0.5);
    
    
            xPidController.setTolerance(0.1);     
            yPidController.setTolerance(0.1); 
            rPidController.setTolerance(1);
        
    
        }
    
        public void moveToState(double goalPositionX, double goalPositionY){
            
            PhotonTrackedTarget target = vision.getTracked();
    
    
            double rOutput = rPidController.calculate(driveTrain.getState().Pose.getRotation().getDegrees(), 170);
            double xOutput = -xPidController.calculate(driveTrain.getState().Pose.getX(), 14.42);
            double yOutput = -yPidController.calculate(driveTrain.getState().Pose.getY(), 4.5);
    
            driveTrain.setControl(requester.withVelocityY(yOutput).withVelocityX(xOutput));
        
        }
    
        @Override
    
        public void periodic(){
            PhotonTrackedTarget target = vision.getTracked();
    
            if (target !=null){ 
                var color = DriverStation.getAlliance();
                if (color.isPresent() && color.get() == DriverStation.Alliance.Blue){
                    currentPose = Constants.autoAlignSide.get("Blue");
            }
            else if (color.isPresent() && color.get() == DriverStation.Alliance.Red){
                currentPose = Constants.autoAlignSide.get("Red");
            }
            //if (robot.)
           // currentPose = Constants.autoAlignSide.get(target)
        }
        
    }
}


