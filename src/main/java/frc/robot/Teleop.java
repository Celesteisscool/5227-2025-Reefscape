package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

public class Teleop {
	public static boolean ALIGNING = false;
	public static double joystickAngle = 0.0;
	static boolean fieldRelative = true;
	
		private static XboxController driverController = Constants.driverController;
	
		public static double deadzones(double input) {
			if (Math.abs(input) < 0.05) {
				return 0;
			} else {
				return input;
			}
		}
	
		public static void driveFunction(double slow) {
			Drivetrain Drivetrain = Constants.drivetrainClass;
			double speedLimiter = 0.95;
	
			if (driverController.getRightTriggerAxis() > 0.5) {
				speedLimiter *= 0.25;
			} 
	
			double xSpeed = (deadzones(driverController.getLeftY())* Constants.maxSwerveSpeed * speedLimiter * slow);
			double ySpeed = (deadzones(driverController.getLeftX()) * Constants.maxSwerveSpeed * speedLimiter * slow);
			double rotSpeed = (deadzones(driverController.getRightX()) * Constants.maxSwerveAngularSpeed * speedLimiter * slow); 
			int POVangle = driverController.getPOV(); 
			
			if (POVangle != -1) { // Drives with the DPad instead of the joystick for perfect 45° angles
				var POVRadians = Math.toRadians(POVangle);
				xSpeed = Math.cos(POVRadians) * -0.25 * speedLimiter;
				ySpeed = Math.sin(POVRadians) * 0.25 * speedLimiter;
				fieldRelative = false; // Forces it to be robot relative
			} else {
				if ((xSpeed > 0) || (ySpeed > 0) || (rotSpeed > 0)) {fieldRelative = true;}
			}

		// Y is left and right.... I guess....
		ALIGNING = false;
		if (driverController.getAButton()) { 
			ySpeed = Constants.visionClass.alginToReef(); 
			ALIGNING = true;
		}

		joystickAngle = Math.toDegrees(Math.atan2(driverController.getLeftX(), driverController.getLeftY()));
		
		Drivetrain.drive(
			Constants.slewRateLimiterX.calculate(xSpeed), 
			Constants.slewRateLimiterY.calculate(ySpeed), 
			Constants.slewRateLimiterRot.calculate(rotSpeed), 
			fieldRelative
		);
	};
		
	public void teleopPeriodic() {
		Constants.elevatorClass.elevatorLogic();
		driveFunction(Constants.elevatorClass.getElevatorSlowdown());
	}
}
