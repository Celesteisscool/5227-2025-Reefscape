package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

public class Teleop {
	private static int aligningSide = 0;
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
			slow = 1;
			Drivetrain Drivetrain = Constants.drivetrainClass;
			double translateSpeedLimiter = 0.95;
			double rotateSpeedLimiter = 0.95;
	
			if (driverController.getRightTriggerAxis() > 0.5) {
				translateSpeedLimiter *= 0.25;
				rotateSpeedLimiter *= 0.1;
			} 

			if (driverController.getLeftBumperButton()) { aligningSide = -1;} 
			else if (driverController.getRightBumperButton()) { aligningSide = 1;} 
			else { aligningSide = 0; }
			Constants.ledClass.AlignSide = aligningSide;
	
			double xSpeed = (deadzones(driverController.getLeftY())* Constants.maxSwerveSpeed * translateSpeedLimiter * slow);
			double ySpeed = (deadzones(driverController.getLeftX()) * Constants.maxSwerveSpeed * translateSpeedLimiter * slow);
			double rotSpeed = (deadzones(driverController.getRightX()) * Constants.maxSwerveAngularSpeed * rotateSpeedLimiter * slow); 
			int POVangle = driverController.getPOV(); 
			
			if (POVangle != -1) { // Drives with the DPad instead of the joystick for perfect 45° angles
				var POVRadians = Math.toRadians(POVangle);
				xSpeed = Math.cos(POVRadians) * -0.25 * translateSpeedLimiter;
				ySpeed = Math.sin(POVRadians) * 0.25 * translateSpeedLimiter;
				fieldRelative = false; // Forces it to be robot relative
			} else {
				if ((xSpeed > 0) || (ySpeed > 0) || (rotSpeed > 0)) {fieldRelative = true;}
			}

			if (aligningSide != 0) {
				xSpeed = Constants.visionClass.alginToReef(aligningSide);
			}
		joystickAngle = Math.toDegrees(Math.atan2(driverController.getLeftX(), driverController.getLeftY()));
		
		double xOutput;
		double yOutput;
		double rotOutput;
		if (driverController.getLeftTriggerAxis() > 0.5) { // FAST DRIVE :)
			Constants.slewRateLimiterX.calculate(xSpeed); // Calculates to update even when pressed
			Constants.slewRateLimiterY.calculate(ySpeed);
			Constants.slewRateLimiterRot.calculate(rotSpeed);
			xOutput = xSpeed;
			yOutput = ySpeed;
			rotOutput = rotSpeed;
		} else {
			xOutput = Constants.slewRateLimiterX.calculate(xSpeed);
			yOutput = Constants.slewRateLimiterY.calculate(ySpeed);
			rotOutput = Constants.slewRateLimiterRot.calculate(rotSpeed);
		}


		Drivetrain.drive(
			xOutput,
			yOutput,
			rotOutput,
			fieldRelative
		);
	};
		
	public void teleopPeriodic() {
		Constants.elevatorClass.elevatorLogic();
		driveFunction(Constants.elevatorClass.getElevatorSlowdown());
	}
}
