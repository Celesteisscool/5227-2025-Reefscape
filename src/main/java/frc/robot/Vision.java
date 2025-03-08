package frc.robot;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Vision {
    public static final PhotonCamera mainCamera = new PhotonCamera("Microsoft_LifeCam_HD-3000");
    /** Returns a joystick-like input for the robot to rotate to the nearest tag */
    public double alginToReef() {
        // Read in relevant data from the Camera
        boolean targetVisible = false;
        double targetYaw = 0.0;
        var results = mainCamera.getAllUnreadResults();
        if (!results.isEmpty()) {
            // Camera processed a new frame since last
            // Get the last one in the list.
            var result = results.get(results.size() - 1);
            if (result.hasTargets()) {
                // At least one AprilTag was seen by the camera
                for (var target : result.getTargets()) {
                    if (testReefTag(target.getFiducialId())) { 
                        targetYaw = target.getYaw();
                        targetVisible = true;
                    }
                }
            }
        }
        // Auto-align when requested
        double output = 0.0;
        if (targetVisible) {
            // Override the driver's turn command with an automatic one that aligns with the tag.
            output = ( targetYaw * 0.25 * Constants.maxSwerveSpeed );
            double slowRange = 20.0;
            if (Math.abs(targetYaw) < slowRange) {
                output = output * (1/(slowRange - Math.abs(targetYaw)));
            }
        }
        
        // Put debug information to the dashboard
        SmartDashboard.putBoolean("Vision Target Visible", targetVisible);    
        
        // Send output
        return output;
    }

    boolean testReefTag(int tagID) {
        boolean isValid = false;
        
        // This is set up like this now for when i add in alliance detection....
        for (int i : Constants.validRedReefIDs) {
            if (i == tagID) { isValid = true; }
        }
        for (int i : Constants.validBlueReefIDs) {
            if (i == tagID) { isValid = true; }
        }

        return isValid;
    }

    public double getClosestYaw() {
        // Read in relevant data from the Camera
        double targetYaw = 0.0;
        var results = mainCamera.getAllUnreadResults();
        double output = 5227;

        if (!results.isEmpty()) {
            // Camera processed a new frame since last
            // Get the last one in the list.
            var result = results.get(results.size() - 1);
            if (result.hasTargets()) {
                // At least one AprilTag was seen by the camera
                for (var target : result.getTargets()) {
                    if (testReefTag(target.getFiducialId())) { 
                        targetYaw = target.getYaw();
                        output = targetYaw;
                    }
                }
            }
        }
        return output;
    }
}

