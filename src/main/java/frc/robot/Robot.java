// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  private final XboxController m_controller = new XboxController(0);
  private final Drivetrain m_swerve = new Drivetrain();
  private SlewRateLimiter m_slewLimiterX = new SlewRateLimiter(0.5);
  private SlewRateLimiter m_slewLimiterY = new SlewRateLimiter(0.5);
  private SlewRateLimiter m_slewLimiterRot = new SlewRateLimiter(0.5);
  
  @Override
  public void autonomousPeriodic() {
    driveWithJoystick(false);
    m_swerve.updateOdometry();
  }

  @Override
  public void robotInit() {
  }

  @Override 
  public void robotPeriodic() {
  }

  @Override
  public void teleopPeriodic() {
    driveWithJoystick(true);
    m_swerve.updateSim();
    SmartDashboard.putNumber("Diff", m_swerve.findWheelDiff());

  }

  private void driveWithJoystick(boolean fieldRelative) {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    var speedLimit = 1 - (m_controller.getRightTriggerAxis() * .5);


    var xSpeed =
        (m_controller.getLeftY()* Drivetrain.kMaxSpeed * speedLimit);

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    var ySpeed =
        (m_controller.getLeftX() * Drivetrain.kMaxSpeed * speedLimit);

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    var rotSpeed =
        (m_controller.getRightX() * Drivetrain.kMaxAngularSpeed * speedLimit) * -1;

    if (m_controller.getPOV() == 0) {
      xSpeed = 0.25;
    } else if (m_controller.getPOV() == 180) {
      xSpeed = -0.25;
    }
    if (m_controller.getPOV() == 90) {
      ySpeed = 0.25;
    } else if (m_controller.getPOV() == 270) {
      ySpeed = -0.25;
    }
    
    

    m_swerve.drive(
      m_slewLimiterX.calculate(xSpeed), 
      m_slewLimiterY.calculate(ySpeed), 
      m_slewLimiterRot.calculate(rotSpeed), 
      fieldRelative, getPeriod()
    );
  }

  @Override
  public void simulationPeriodic() {
  }

  public void testPeriodic() {
    if (m_controller.getXButton()) {
      m_swerve.setMotorVoltage(0.45);
    }
  }
}
