// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private final XboxController m_Drivercontroller = new XboxController(0);
  private final XboxController m_Elevatorcontroller = new XboxController(1);
  private final Drivetrain m_swerve = new Drivetrain();
  private final Elevator m_Elevator = new Elevator();

  
  @Override
  public void autonomousInit() {
    m_swerve.getAutonomousCommand().schedule();
  }

  @Override
  public void autonomousPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void robotInit() {
  }

  @Override 
  public void robotPeriodic() {
  }

  @Override
  public void teleopPeriodic() {
    m_swerve.driveWithJoystick(true, m_Drivercontroller, m_swerve, getPeriod());
    m_Elevator.elevatorLogic(m_Elevatorcontroller);
  }

  

  @Override
  public void simulationPeriodic() {
  }

  public void testPeriodic() {
    if (m_Drivercontroller.getXButton()) {
      m_swerve.setMotorVoltage(0.45);
    }
  }
}
