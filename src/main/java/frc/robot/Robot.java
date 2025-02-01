// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private final XboxController driverController = new XboxController(0);
  private final XboxController elevatorController = new XboxController(1);
  private Drivetrain swerveDrivetrain;
  private final Elevator elevator = new Elevator();
  private Command autoCommand;
  
  @Override
  public void autonomousInit() {
    autoCommand = swerveDrivetrain.getAutonomousCommand();
    if (autoCommand != null) {
      autoCommand.schedule();
    }
  }

  @Override
  public void teleopInit() {
    if (autoCommand != null) {
      autoCommand.cancel();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void robotInit() {
    swerveDrivetrain = new Drivetrain();
    CommandScheduler.getInstance().registerSubsystem(swerveDrivetrain);
  }

  @Override 
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); // Should have read docs :(
  }

  @Override
  public void teleopPeriodic() {
    swerveDrivetrain.driveWithJoystick(true, driverController, swerveDrivetrain, getPeriod());
    elevator.elevatorLogic(elevatorController);
  }

  @Override
  public void simulationPeriodic() {}

  public void testPeriodic() {
    if (driverController.getXButton()) {
      swerveDrivetrain.setMotorVoltage(0.45);
    }
  }
}
