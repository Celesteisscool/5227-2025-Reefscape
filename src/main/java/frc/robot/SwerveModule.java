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
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class SwerveModule {
  private static final double kModuleMaxAngularVelocity = Drivetrain.maxSwerveAngularSpeed;
  private static final double kModuleMaxAngularAcceleration =
  2 * Math.PI; // radians per second squared
  
  private final SparkMax m_driveMotor; 
  private final RelativeEncoder m_driveEncoder;
  private final SparkMax m_rotationMotor;
  private final CANcoder m_rotationEncoder;  

  // Gains are for example purposes only - must be determined for your own robot!
  private final PIDController m_drivePIDController = new PIDController(0, 0, 0);

  // Desired state
  private SwerveModuleState m_desiredState;
  // Gains are for example purposes only - must be determined for your own robot!
  private final ProfiledPIDController m_rotationPIDController =
      new ProfiledPIDController(
          7.25,
          0,
          0,
          new TrapezoidProfile.Constraints(
              kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration));
              
  // Gains are for example purposes only - must be determined for your own robot!
  private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(1, 3);
  private final SimpleMotorFeedforward m_rotationFeedforward = new SimpleMotorFeedforward(0.45, 0); 

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
    m_driveMotor = new SparkMax(driveMotorChannel, MotorType.kBrushless);
    m_rotationMotor = new SparkMax(rotationMotorChannel, MotorType.kBrushless);
    m_rotationEncoder = new CANcoder(rotationEncoderChannel);
    m_driveEncoder = m_driveMotor.getEncoder();
    
    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_rotationPIDController.enableContinuousInput(-Math.PI, Math.PI);

    m_rotationEncoder.setPosition(0);
  }

  public double getDriveSpeedMetersPerSecond() {
    double RawRPM = m_driveMotor.getEncoder().getVelocity();
    RawRPM = (RawRPM / 6.12);
    double RawRPS = RawRPM / 60;
    double Conversion = (2 * Math.PI * 0.0508);
    double MPS = RawRPS * Conversion;
    return MPS;
  }

  /* Returns the distance traveled by the drive encoder in meters */
  public double getDriveDistanceMeters() {
    return ((m_driveEncoder.getPosition()/6.12) * (2*Math.PI*0.0508)); 
  }

  /* Returns the angle traveled by the rotation encoder in radians */
  public double getRotationEncoderPosition() {
    return (m_rotationEncoder.getPosition().getValueAsDouble() * (2 * Math.PI)); // Returns the angle in radians
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
    return m_desiredState;
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
    m_rotationPIDController.setPID(P, I, D);
  }

  public void moveWithVoltage(double volts) {
    m_rotationMotor.setVoltage(volts);
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    var encoderRotation = new Rotation2d(getRotationEncoderPosition());

    m_desiredState = desiredState;
    // Optimize the reference state to avoid spinning further than 90 degrees
    desiredState.optimize(encoderRotation);

    // Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
    // direction of travel that can occur when modules change directions. This results in smoother
    // driving.
    desiredState.cosineScale(encoderRotation);

    // Calculate the drive output from the drive PID controller.

    final double driveOutput =
        m_drivePIDController.calculate(m_driveEncoder.getVelocity(), desiredState.speedMetersPerSecond);

    final double driveFeedforward = m_driveFeedforward.calculate(desiredState.speedMetersPerSecond);

    // Calculate the rotation motor output from the rotation PID controller.
    final double rotateOutput =
        m_rotationPIDController.calculate(
            getRotationEncoderPosition(), desiredState.angle.getRadians());

    final double rotationFeedforward =
        m_rotationFeedforward.calculate(m_rotationPIDController.getSetpoint().velocity);

    m_driveMotor.setVoltage(driveOutput + driveFeedforward);
    m_rotationMotor.setVoltage(rotateOutput + rotationFeedforward);
  }
}
