// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;

public class Robot extends TimedRobot {
  Auto auto = new Auto();
  Teleop teleop = new Teleop();
  public int LEDMODE = 0;
  @Override
  public void autonomousInit() { auto.tempAutoCodeInit(); }
  
  @Override
  public void robotInit() {}

  @Override
  public void disabledInit() {
    LEDMODE = 0;
  }

  @Override 
  public void disabledPeriodic() { 
    
  }

  @Override 
  public void robotPeriodic() {
  }

  @Override
  public void autonomousPeriodic() { auto.tempAutoCodePeriodic(); }

  @Override
  public void teleopInit() {
    LEDMODE = 1;
  }
  @Override
  public void teleopPeriodic() {
    teleop.teleopPeriodic(); 
  }
}
