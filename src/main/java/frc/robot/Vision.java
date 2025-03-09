package frc.robot;

import org.photonvision.PhotonCamera;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Vision {
    public static final PhotonCamera leftCamera  = new PhotonCamera("leftCamera");
    public static final PhotonCamera rightCamera = new PhotonCamera("rightCamera");
    private PIDController alignPIDController = new PIDController(0, 0, 0); // Tune these fr fr
    /** Returns a joystick-like input for the robot to rotate to the nearest tag */
    public Double alginToReef(int side) {
        PhotonCamera camera = null;
        // Read in relevant data from the Camera
        if (side == 1) {
            camera = rightCamera;
        } else if (side == -1) {
            camera = leftCamera;
        }

        Double targetYaw = getClosestYaw(camera);

        // Auto-align when requested
        Double output = null;
        if (targetYaw != null) {
            // Override the driver's move command with an automatic one that aligns with the tag.
            alignPIDController.setSetpoint(0);
            output = alignPIDController.calculate(targetYaw);
        }
        
        // Put debug information to the dashboard
        SmartDashboard.putBoolean("Vision Target Visible", (targetYaw != null));    
        
        // Send output
        return output;
    }

    /** tests if a tag is a reef tag */
    boolean testReefTag(int tagID) {
        boolean isValid = false;
        for (int i : Constants.validRedReefIDs)  { if (i == tagID) { isValid = true; }}
        for (int i : Constants.validBlueReefIDs) { if (i == tagID) { isValid = true; }}
        return isValid;
    }

    /** Gets the closest reef tag and returns its yaw */
    public Double getClosestYaw(PhotonCamera camera) {
        if (camera == null) { return null; }
        // Read in relevant data from the Camera
        double targetYaw = 0.0;
        var results = camera.getAllUnreadResults();
        Double output = null;

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

    /** Switches cameras in and out of driver mode. cameraToUse is for april tags */
    public void switchCameraModes(int cameraToUse) {
        if (cameraToUse == -1) {
            leftCamera.setDriverMode(false);
            rightCamera.setDriverMode(true);
        } else if (cameraToUse == 1) {
            leftCamera.setDriverMode(true);
            rightCamera.setDriverMode(false);
        } else if (cameraToUse == 0) {
            leftCamera.setDriverMode(false);
            rightCamera.setDriverMode(false);
        }
    }

}

