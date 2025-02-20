package frc.robot;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision {
    public static final PhotonCamera mainCamera = new PhotonCamera("Camera");

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

    /** Returns a joystick-like input for the robot to rotate to the nearest tag */
    public double getCalculatedRotateOutput() {
        if (getTarget() != null) {
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