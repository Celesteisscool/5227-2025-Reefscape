package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;

public class Teleop {
    public Teleop() {
        
    }

    public Command driveCommand() {
        return new RunCommand(() -> {
            XboxController driverController = Constants.driverController;
            Drivetrain Drivetrain = Constants.swerveDrivetrain;

            double speedLimiter = 1 - (driverController.getRightTriggerAxis() * .5);
            double xSpeed = (driverController.getLeftY()* Constants.maxSwerveSpeed * speedLimiter);
            double ySpeed = (driverController.getLeftX() * Constants.maxSwerveSpeed * speedLimiter);
            double rotSpeed = (driverController.getRightX() * Constants.maxSwerveAngularSpeed * speedLimiter) * -1; //invert so that turning is more natural
            int POV = driverController.getPOV();
        
            boolean fieldRelative = true;
            if (POV != -1) { // Drives with the DPad instead of the joystick for perfect 45° angles
              var POVRadians = Math.toRadians(POV);
              xSpeed = Math.cos(POVRadians) * 0.25;
              ySpeed = Math.sin(POVRadians) * 0.25;
              fieldRelative = false; // Forces it to be robot relative
            }
        
            if (driverController.getAButton()) {   
              rotSpeed = Constants.vision.getCalculatedRotateOutput();
            }
            
            Drivetrain.drive(
              Constants.slewRateLimiterX.calculate(xSpeed), 
              Constants.slewRateLimiterY.calculate(ySpeed), 
              Constants.slewRateLimiterRot.calculate(rotSpeed), 
              fieldRelative, // i need getperdio here....
            );
          });
    };


    public void startTeleop() {

        CommandScheduler.getInstance().schedule();
    }
}
