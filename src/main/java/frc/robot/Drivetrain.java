// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.XboxController;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain {
  
  public static final double kMaxSpeed = 1; // 3 meters per second
  public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

  private final Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
  private final Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
  private final Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
  private final Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);

  private final SwerveModule m_frontLeft = new SwerveModule(5, 6, 11);
  private final SwerveModule m_frontRight = new SwerveModule(3,4,10);
  private final SwerveModule m_backLeft = new SwerveModule(7,8,12);
  private final SwerveModule m_backRight = new SwerveModule(1,2,9);

    
  private SlewRateLimiter m_slewLimiterX = new SlewRateLimiter(0.5);
  private SlewRateLimiter m_slewLimiterY = new SlewRateLimiter(0.5);
  private SlewRateLimiter m_slewLimiterRot = new SlewRateLimiter(0.5);

  private final Pigeon2 m_gyro = new Pigeon2(32);

  StructArrayPublisher<SwerveModuleState> Desired = NetworkTableInstance.getDefault().getStructArrayTopic("Desired", SwerveModuleState.struct).publish();
  StructArrayPublisher<SwerveModuleState> Current = NetworkTableInstance.getDefault().getStructArrayTopic("Current", SwerveModuleState.struct).publish();

  private final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(
          m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  private final SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(
          m_kinematics,
          m_gyro.getRotation2d(),
          new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
          });

  public Drivetrain() {
    m_gyro.reset();
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(
      double xSpeed, double ySpeed, double rot, boolean fieldRelative, double periodSeconds) {
    var swerveModuleStates =m_kinematics.toSwerveModuleStates(ChassisSpeeds.discretize(fieldRelative
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d()) // Feild reliative
                    : new ChassisSpeeds(xSpeed, ySpeed, rot), // Absolute
                periodSeconds));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
    updateSim();
  }

  public void updateSim() {
    SwerveModuleState[] DesiredS = new SwerveModuleState[] {
          m_frontLeft.getDesiredState(),
          m_frontRight.getDesiredState(),
          m_backLeft.getDesiredState(),
          m_backRight.getDesiredState()
    };
    Desired.set(DesiredS);

    SwerveModuleState[] CurrentS = new SwerveModuleState[] {
      m_frontLeft.getState(),
      m_frontRight.getState(),
      m_backLeft.getState(),
      m_backRight.getState()
  };
  Current.set(CurrentS);
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_odometry.update(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_backLeft.getPosition(),
          m_backRight.getPosition()
        });
  }

  public double findWheelDiff() {
    return (m_frontRight.getDesiredState().angle.getRadians() - m_frontRight.getState().angle.getRadians());
  }

  public void setMotorVoltage(double volts) {
    m_frontLeft.moveWithVoltage(volts);
    m_frontRight.moveWithVoltage(volts);
    m_backLeft.moveWithVoltage(volts);
    m_backRight.moveWithVoltage(volts);
  }



  public void driveWithJoystick(boolean fieldRelative, XboxController m_Drivercontroller, Drivetrain m_swerve, double period) {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    var speedLimit = 1 - (m_Drivercontroller.getRightTriggerAxis() * .5);


    var xSpeed =
        (m_Drivercontroller.getLeftY()* Drivetrain.kMaxSpeed * speedLimit);

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    var ySpeed =
        (m_Drivercontroller.getLeftX() * Drivetrain.kMaxSpeed * speedLimit);

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    var rotSpeed =
        (m_Drivercontroller.getRightX() * Drivetrain.kMaxAngularSpeed * speedLimit) * -1;

    if (m_Drivercontroller.getPOV() == 0) {
      xSpeed = 0.25;
    } else if (m_Drivercontroller.getPOV() == 180) {
      xSpeed = -0.25;
    }
    if (m_Drivercontroller.getPOV() == 90) {
      ySpeed = 0.25;
    } else if (m_Drivercontroller.getPOV() == 270) {
      ySpeed = -0.25;
    }
    
    

    m_swerve.drive(
      m_slewLimiterX.calculate(xSpeed), 
      m_slewLimiterY.calculate(ySpeed), 
      m_slewLimiterRot.calculate(rotSpeed), 
      fieldRelative, period
    );
  }
}
