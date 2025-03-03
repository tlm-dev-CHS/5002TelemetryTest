package frc.robot;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class autoAlign extends Command {
    private final PhotonCamera camera;
    private final CommandSwerveDrivetrain swerveDrive;
    private final SwerveRequest.FieldCentric requester;

    private final PIDController rotationController;
    private final PIDController xPidController;
    private final PIDController yPidController;

    private final int targetID;
    private final Timer lostTargetTimer = new Timer();

    private static final double LOST_TARGET_TIMEOUT = 0.5; 

    public autoAlign(CommandSwerveDrivetrain swerveDrive, int targetID) {
        this.swerveDrive = swerveDrive;
        this.targetID = targetID;
        addRequirements(swerveDrive);

        camera = new PhotonCamera(OperatorConstants.cameraName);
        requester = new SwerveRequest.FieldCentric();

        // PID Controllers with Tolerance
        rotationController = new PIDController(0.5, 0.0, 0.0);
        xPidController = new PIDController(0.5, 0.0, 0.0);
        yPidController = new PIDController(0.5, 0.0, 0.0);

        rotationController.setTolerance(0.05); 
        xPidController.setTolerance(0.02);     
        yPidController.setTolerance(0.02);    
    }

    @Override
    public void initialize() {
        lostTargetTimer.reset();
        lostTargetTimer.start();
    }

    @Override
    public void execute() {
        PhotonPipelineResult result = camera.getLatestResult();
        boolean hasTargets = result.hasTargets();

        if (hasTargets) {
            var target = result.getBestTarget();
            if (target.getFiducialId() == targetID) {
                Transform3d bestCameraToTarget = target.getBestCameraToTarget();
                double distance = OperatorConstants.distanceToTag;  

                double rError = bestCameraToTarget.getRotation().getZ();
                double yError = bestCameraToTarget.getY();
                double xError = bestCameraToTarget.getX() - distance;

                double xOutput = xPidController.calculate(xError, 0);
                double yOutput = yPidController.calculate(yError, 0);
                double rOutput = rotationController.calculate(rError, 0);

                swerveDrive.setControl(requester.withVelocityX(xOutput)
                                                .withVelocityY(yOutput)
                                                .withRotationalRate(rOutput));

                lostTargetTimer.reset(); 
            }
        }

        if (!hasTargets && lostTargetTimer.hasElapsed(LOST_TARGET_TIMEOUT)) {
            end(true);
        }
    }
//hey
    @Override
    public boolean isFinished() {
        PhotonPipelineResult result = camera.getLatestResult();
        if (result.hasTargets()) {
            var target = result.getBestTarget();
            if (target.getFiducialId() == targetID) {
                Transform3d bestCameraToTarget = target.getBestCameraToTarget();
                double xError = bestCameraToTarget.getX() - OperatorConstants.distanceToTag;
                double yError = bestCameraToTarget.getY();
                double rError = bestCameraToTarget.getRotation().getZ();

                return xPidController.atSetpoint() &&
                       yPidController.atSetpoint() &&
                       rotationController.atSetpoint();
            }
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        swerveDrive.setControl(requester.withVelocityX(0)
                                        .withVelocityY(0)
                                        .withRotationalRate(0));
    }
}
