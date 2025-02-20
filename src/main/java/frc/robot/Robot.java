// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  
  @Override
  public void autonomousInit() {
  }
  
  @Override
  public void robotInit() {}

  @Override 
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); // Should have read docs :(
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {
    Teleop.runTeleop();
  }
}
