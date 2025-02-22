package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

public class Teleop {
	public Teleop() {} //This makes compiler happy

	public static void driveFunction() {
		XboxController driverController = Constants.driverController;
		Drivetrain Drivetrain = Constants.drivetrainClass;
		
		double speedLimiter = 1 - (driverController.getRightTriggerAxis() * .5);
		double xSpeed = (driverController.getLeftY()* Constants.maxSwerveSpeed * speedLimiter);
		double ySpeed = (driverController.getLeftX() * Constants.maxSwerveSpeed * speedLimiter);
		double rotSpeed = (driverController.getRightX() * Constants.maxSwerveAngularSpeed * speedLimiter) * -1; //invert so that turning is more natural
		int POVangle = driverController.getPOV(); 
		boolean fieldRelative = true;
		if (POVangle != -1) { // Drives with the DPad instead of the joystick for perfect 45° angles
			var POVRadians = Math.toRadians(POVangle);
			xSpeed = Math.cos(POVRadians) * 0.25;
			ySpeed = Math.sin(POVRadians) * 0.25;
			fieldRelative = false; // Forces it to be robot relative
		}

		// Y is left and right.... I guess....
		if (driverController.getAButton()) { ySpeed = Constants.visionClass.alginToReef(); }
		
		Drivetrain.drive(
			Constants.slewRateLimiterX.calculate(xSpeed), 
			Constants.slewRateLimiterY.calculate(ySpeed), 
			Constants.slewRateLimiterRot.calculate(rotSpeed), 
			fieldRelative
		);
	};
		
	public void teleopPeriodic() {
		driveFunction();
	}
}
