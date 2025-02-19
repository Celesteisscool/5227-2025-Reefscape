package frc.robot;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform3d;

public class Vision {

    // Vision.Java Constants 
    public static final PhotonCamera mainCamera = new PhotonCamera("Camera"); // No clue what its called, will probably change once actually deployed
    
    public Vision() {
        mainCamera.setPipelineIndex(0);
        PhotonCamera.setVersionCheckEnabled(false);
    }  

    public PhotonTrackedTarget getNearestTag() {
        var results = mainCamera.getAllUnreadResults();
        PhotonTrackedTarget outTarget = null;
        double bestDistance = -1;
        if (!results.isEmpty()) {
            var latestResult  = results.get(results.size() - 1);
            if (latestResult.hasTargets()) { 
                for (var target : latestResult.getTargets()) {
                    Transform3d transform = target.getBestCameraToTarget();
                    double distance = Math.sqrt(Math.pow(transform.getX(), 2) + Math.pow(transform.getY(), 2));                
                    if (distance > bestDistance) {
                        outTarget = target;
                        bestDistance = distance;
                    }
                }
            }
        }
        return outTarget;
    }

    public PhotonTrackedTarget getTarget() {
        var results = mainCamera.getAllUnreadResults();
        PhotonTrackedTarget target = null;
        if (!results.isEmpty()) {
            var latestResult  = results.get(results.size() - 1);
            if (latestResult.hasTargets()) { 
                target = latestResult.getBestTarget();
            }
        }
        return target;


    }


    /** Returns a joystick-like input for the robot to drive */
    public double getCalculatedRotateOutput() {
        if (getNearestTag() != null) {
            double yaw = getTarget().getYaw();
            double calculatedTurn = -1.0 * yaw * Constants.maxSwerveAngularSpeed;
            System.out.println(yaw);
            return calculatedTurn;
        }
        else {
            return 0;
        }

    }
    
}
