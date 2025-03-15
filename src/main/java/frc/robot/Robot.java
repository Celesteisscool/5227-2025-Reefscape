// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;

public class Robot extends TimedRobot {
  Auto auto = new Auto();
  Teleop teleop = new Teleop();

  @Override
  public void robotInit() {}
  
  @Override 
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() { auto.autoInit(); }
  
  @Override
  public void autonomousPeriodic() { 
    auto.runSelectedAuto(); 
  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {
    teleop.teleopPeriodic(); 
  }

  @Override
  public void disabledInit() {
  }

  @Override 
  public void disabledPeriodic() { 
    Constants.ledClass.setLEDPurpleGold();
    Constants.ledClass.update();
  }
}
