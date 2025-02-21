package frc.robot;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Vision {
    public static final PhotonCamera mainCamera = new PhotonCamera("Microsoft_LifeCam_HD-3000");
    public SlewRateLimiter slewRateLimiter = new SlewRateLimiter(1);
    /** Returns a joystick-like input for the robot to rotate to the nearest tag */
    public double getCalculatedRotateOutput() {
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
                    if (target.getFiducialId() == 7) {
                        // Found Tag 7, record its information
                        targetYaw = target.getYaw();
                        targetVisible = true;
                    }
                }
            }
        }
        // Auto-align when requested
        double output = 0.0;
        if (targetVisible) {
            // Driver wants auto-alignment to tag 7
            // And, tag 7 is in sight, so we can turn toward it.
            // Override the driver's turn command with an automatic one that turns toward the tag.
            output = ( targetYaw * 0.5 * Constants.maxSwerveAngularSpeed );
        }

        // Put debug information to the dashboard
        SmartDashboard.putBoolean("Vision Target Visible", targetVisible);    
        
        // Send output
        return slewRateLimiter.calculate(output);
    }
}