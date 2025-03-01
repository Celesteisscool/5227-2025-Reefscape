package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

public class Teleop {
	public static boolean ALIGNING = false;
	public static double joystickAngle = 0.0;
	public Teleop() {} //This makes compiler happy
	private static XboxController driverController = Constants.driverController;
	private static Joystick elevatorController = new Joystick(1);

	public static double deadzones(double input) {
		if (Math.abs(input) < 0.05) {
			return 0;
		} else {
			return input;
		}
	}

	public static void driveFunction() {
		Drivetrain Drivetrain = Constants.drivetrainClass;
		

		double speedLimiter = 1 - (driverController.getRightTriggerAxis() * .5);
		double xSpeed = (deadzones(driverController.getLeftY())* Constants.maxSwerveSpeed * speedLimiter);
		double ySpeed = (deadzones(driverController.getLeftX()) * Constants.maxSwerveSpeed * speedLimiter);
		double rotSpeed = (deadzones(driverController.getRightX()) * Constants.maxSwerveAngularSpeed * speedLimiter) * -1; //invert so that turning is more natural
		int POVangle = driverController.getPOV(); 
		boolean fieldRelative = true;
		if (POVangle != -1) { // Drives with the DPad instead of the joystick for perfect 45° angles
			var POVRadians = Math.toRadians(POVangle);
			xSpeed = Math.cos(POVRadians) * 0.25;
			ySpeed = Math.sin(POVRadians) * 0.25;
			fieldRelative = false; // Forces it to be robot relative
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
		driveFunction();
		Constants.elevatorClass.elevatorLogic(elevatorController);

		if (driverController.getYButton()) {
			Constants.drivetrainClass.setPID();
		}
	}
}
