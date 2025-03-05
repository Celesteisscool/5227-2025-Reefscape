// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class SwerveModule {
  private static final double moduleMaxAngularVelocity = Constants.maxSwerveSpeed;
  private static final double moduleMaxAngularAcceleration = 2 * Math.PI; // radians per second squared
  
  private final SparkMax driveMotor; 
  private final RelativeEncoder driveEncoder;
  private final SparkMax rotationMotor;
  private final CANcoder rotationEncoder;  

  // Gains are for example purposes only - must be determined for your own robot!
  private final PIDController drivePIDController = new PIDController(0, 0, 0);

  // Desired state
  private SwerveModuleState desiredState;
  // Gains are for example purposes only - must be determined for your own robot!
  private double rotateP = 0.7;
  private final ProfiledPIDController rotationPIDController =
      new ProfiledPIDController(rotateP,0,0,
          new TrapezoidProfile.Constraints(
              moduleMaxAngularVelocity, moduleMaxAngularAcceleration));
              
  // Gains are for example purposes only - must be determined for your own robot!
  private final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(1, 3);
  /**
   * Constructs a SwerveModule with a drive motor, rotation motor, drive encoder and rotation encoder.
   *
   * @param driveMotorChannel CAN id for the drive motor.
   * @param rotationMotorChannel CAN id for the rotation motor.
   * @param rotationEncoderChannel CAN id for the rotation encoder.
   */
  public SwerveModule(
      int driveMotorChannel,
      int rotationMotorChannel,
      int rotationEncoderChannel) {
    driveMotor = new SparkMax(driveMotorChannel, MotorType.kBrushless);
    rotationMotor = new SparkMax(rotationMotorChannel, MotorType.kBrushless);
    rotationEncoder = new CANcoder(rotationEncoderChannel);
    driveEncoder = driveMotor.getEncoder();
    
    // Limit the PID Controller's input range between -pi and pi and set the input to be continuous.
    rotationPIDController.enableContinuousInput(-Math.PI, Math.PI);

    // Reset the wheels, as we manually align them.
    rotationEncoder.setPosition(0);
  }

  public double getDriveSpeedMetersPerSecond() {
    double RawRPM =driveMotor.getEncoder().getVelocity();
    RawRPM = (RawRPM / 6.12);
    double RawRPS = RawRPM / 60;
    double Conversion = (2 * Math.PI * 0.0508);
    double MPS = RawRPS * Conversion;
    return MPS;
  }

  /* Returns the distance traveled by the drive encoder in meters */
  public double getDriveDistanceMeters() {
    return ((driveEncoder.getPosition()/6.12) * (2*Math.PI*0.0508)); 
  }

  /* Returns the angle traveled by the rotation encoder in radians */
  public double getRotationEncoderPosition() {
    return (rotationEncoder.getPosition().getValueAsDouble() * (2 * Math.PI)); // Returns the angle in radians
  } 

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        getDriveSpeedMetersPerSecond(), new Rotation2d(getRotationEncoderPosition()));
  }

  public SwerveModuleState getDesiredState() {
    return desiredState;
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition returnPosition() {
    return new SwerveModulePosition(
        getDriveDistanceMeters(), new Rotation2d(getRotationEncoderPosition()));
  }

  public void updatePID(double P, double I, double D) {
    rotationPIDController.setPID(P, I, D);
  }

  public void moveWithVoltage(double volts) {
    rotationMotor.setVoltage(volts);
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredStateInput Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredStateInput) {
    var encoderRotation = new Rotation2d(getRotationEncoderPosition());

    desiredState = desiredStateInput;
    // Optimize the reference state to avoid spinning further than 90 degrees
    desiredStateInput.optimize(encoderRotation);

    // Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
    // direction of travel that can occur when modules change directions. This results in smoother
    // driving.
    desiredStateInput.cosineScale(encoderRotation);

    // Calculate the drive output from the drive PID controller.
    final double driveOutput =
        drivePIDController.calculate(driveEncoder.getVelocity(), desiredStateInput.speedMetersPerSecond);
    final double calculatedDriveFeedForward = driveFeedforward.calculate(desiredStateInput.speedMetersPerSecond);

    // Calculate the rotation motor output from the rotation PID controller.
    final double rotateOutput =
        rotationPIDController.calculate(getRotationEncoderPosition(), desiredStateInput.angle.getRadians());

    driveMotor.setVoltage(driveOutput + calculatedDriveFeedForward);
    rotationMotor.set(rotateOutput);
  }


  public void setPID() {
    double P = SmartDashboard.getNumber("Swerve P", rotateP);
    double I = SmartDashboard.getNumber("Swerve I", 0);
    double D = SmartDashboard.getNumber("Swerve D", 0);
    rotationPIDController.setPID(P, I, D);
  }
}
