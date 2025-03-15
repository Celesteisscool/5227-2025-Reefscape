// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.Pigeon2; // Holy import batman
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain {
  
  public static final double maxSwerveSpeed        = Constants.maxSwerveSpeed;
  public static final double maxSwerveAngularSpeed = Constants.maxSwerveAngularSpeed;

  private final Translation2d frontLeftLocation  = Constants.frontLeftLocation;
  private final Translation2d frontRightLocation = Constants.frontRightLocation;
  private final Translation2d backLeftLocation   = Constants.backLeftLocation;
  private final Translation2d backRightLocation  = Constants.backRightLocation;

  private final SwerveModule frontLeft  = new SwerveModule(Constants.frontLeftIDs[0],  Constants.frontLeftIDs[1],  Constants.frontLeftIDs[2]);
  private final SwerveModule frontRight = new SwerveModule(Constants.frontRightIDs[0], Constants.frontRightIDs[1], Constants.frontRightIDs[2]);
  private final SwerveModule backLeft   = new SwerveModule(Constants.backLeftIDs[0],   Constants.backLeftIDs[1],   Constants.backLeftIDs[2]);
  private final SwerveModule backRight  = new SwerveModule(Constants.backRightIDs[0],  Constants.backRightIDs[1],  Constants.backRightIDs[2]);

  public static final Pigeon2 pigeon2 = new Pigeon2(Constants.gyroID);

  StructArrayPublisher<SwerveModuleState> DesiredState = NetworkTableInstance.getDefault().getStructArrayTopic("Desired State", SwerveModuleState.struct).publish();
  StructArrayPublisher<SwerveModuleState> CurrentState = NetworkTableInstance.getDefault().getStructArrayTopic("Current State", SwerveModuleState.struct).publish();
  StructPublisher<Pose2d> robotPose = NetworkTableInstance.getDefault().getStructTopic("MyPose", Pose2d.struct).publish();


  private final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(
          frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

  public final SwerveDriveOdometry odometry =
      new SwerveDriveOdometry(
          kinematics,
          getGyroRotation2d(),
          new SwerveModulePosition[] {
            frontLeft.returnPosition(),
            frontRight.returnPosition(),
            backLeft.returnPosition(),
            backRight.returnPosition()
          });

  public Drivetrain() {
    pigeon2.setYaw(180);  
  }

  public Rotation2d getGyroRotation2d() {
    return pigeon2.getRotation2d();
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rotSpeed Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(
      double xSpeed, double ySpeed, double rotSpeed, boolean fieldRelative) {
    
    SwerveModuleState[] swerveModuleStates;
    if (fieldRelative) { swerveModuleStates = kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotSpeed, pigeon2.getRotation2d())); }
    else { swerveModuleStates = kinematics.toSwerveModuleStates(new ChassisSpeeds(xSpeed, ySpeed, rotSpeed)); }
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, maxSwerveSpeed);
    setStates(swerveModuleStates);
    updateDashboard();
  }

  public void driveFieldRelativeFromSpeed(ChassisSpeeds speeds) {
    var swerveModuleStates  = kinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, maxSwerveSpeed);
    updateDashboard();
  }

  private void setStates(SwerveModuleState[] states) {
    frontLeft.setDesiredState(states[0]);
    frontRight.setDesiredState(states[1]);
    backLeft.setDesiredState(states[2]);
    backRight.setDesiredState(states[3]);
  }

  /** Updates the dashboard with the current state of the swerve modules */
  public void updateDashboard() {
    SwerveModuleState[] DesiredS = new SwerveModuleState[] {
          frontLeft.getDesiredState(),
          frontRight.getDesiredState(),
          backLeft.getDesiredState(),
          backRight.getDesiredState()
    };
    DesiredState.set(DesiredS);

    SwerveModuleState[] CurrentS = new SwerveModuleState[] {
      frontLeft.getState(),
      frontRight.getState(),
      backLeft.getState(),
      backRight.getState()
    };
    CurrentState.set(CurrentS);


    robotPose.set(odometry.getPoseMeters());
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    odometry.update(
        getGyroRotation2d(),
        new SwerveModulePosition[] {
          frontLeft.returnPosition(),
          frontRight.returnPosition(),
          backLeft.returnPosition(),
          backRight.returnPosition()
        });
  }  

  public void resetOdometry(Pose2d pose) {
    odometry.resetPose(pose);
  }

  public void setPID() {
    frontLeft.setPID();
    frontRight.setPID();
    backLeft.setPID();
    backRight.setPID();
  }
}
