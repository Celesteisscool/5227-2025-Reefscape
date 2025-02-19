// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.Pigeon2; // Holy import batman
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.filter.SlewRateLimiter;
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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain extends SubsystemBase {
  
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

  private final PPHolonomicDriveController pathplannerDriveController = new PPHolonomicDriveController(
        new PIDConstants(5.0, 0.0, 0.0), // DEFINITELY gotta be changed.
        new PIDConstants(5.0, 0.0, 0.0)
  );

  private final Pigeon2 pigeon2 = new Pigeon2(Constants.gyroID);

  StructArrayPublisher<SwerveModuleState> DesiredState = NetworkTableInstance.getDefault().getStructArrayTopic("Desired State", SwerveModuleState.struct).publish();
  StructArrayPublisher<SwerveModuleState> CurrentState = NetworkTableInstance.getDefault().getStructArrayTopic("Current State", SwerveModuleState.struct).publish();

  private final SendableChooser<Command> autoChooser;

  private final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(
          frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

  private final SwerveDriveOdometry odometry =
      new SwerveDriveOdometry(
          kinematics,
          getGyroRotation2d(),
          new SwerveModulePosition[] {
            frontLeft.returnPosition(),
            frontRight.returnPosition(),
            backLeft.returnPosition(),
            backRight.returnPosition()
          });

  private Pose2d getPose() {
    updateOdometry(); //Update just in case
    return odometry.getPoseMeters();
  }

  // I love one liners <3 (all of this is for pathplanner)
  private void resetPose(Pose2d pose) { odometry.resetPosition(getGyroRotation2d(), returnWheelPositions(), pose); }
  private ChassisSpeeds getRobotRelativeSpeeds() { return kinematics.toChassisSpeeds(returnWheelStates()); }
  
  private void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) { 
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.2);
    SwerveModuleState[] targetStates = kinematics.toSwerveModuleStates(targetSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, maxSwerveSpeed);
    setStates(targetStates);
   }

  public Drivetrain() {
    pigeon2.reset();
    RobotConfig robotConfig = null; // Thank you team 6517 for this brilliant workaround o7
    try{
      robotConfig = RobotConfig.fromGUISettings();
    } catch (Exception error) {
      // Handle exception IF needed
      error.addSuppressed(error);
    }
    
    AutoBuilder.configure(
      this::getPose, 
      this::resetPose, 
      this::getRobotRelativeSpeeds, 
      (speeds, feedforwards) -> driveRobotRelative(speeds), 
      pathplannerDriveController,
      robotConfig, 
      () -> {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
          return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
      },
      this // Reference to this subsystem to set requirements
      );

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Mode", autoChooser);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public Rotation2d getGyroRotation2d() {
    return pigeon2.getRotation2d().plus(new Rotation2d(Math.PI));
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
    var swerveModuleStates = kinematics.toSwerveModuleStates(
      ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, pigeon2.getRotation2d()), // Felid relative
      periodSeconds); // do you not use discretize??????
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, maxSwerveSpeed);
    setStates(swerveModuleStates);
    updateDashboard();
  }
// wait let me update code
  private void setStates(SwerveModuleState[] states) {
    frontLeft.setDesiredState(states[0]);
    frontRight.setDesiredState(states[1]);
    backLeft.setDesiredState(states[2]);
    backRight.setDesiredState(states[3]);
  }

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
  }

  private SwerveModuleState[] returnWheelStates() {
    return new SwerveModuleState[] {
      frontLeft.getState(),
      frontRight.getState(),
      backLeft.getState(),
      backRight.getState()
    };
  }

  private SwerveModulePosition[] returnWheelPositions() {
    return new SwerveModulePosition[] {
      frontLeft.returnPosition(),
      frontRight.returnPosition(),
      backLeft.returnPosition(),
      backRight.returnPosition()
    };
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
  /** This is for system identification */
  public void setMotorVoltage(double volts) { 
    frontLeft.moveWithVoltage(volts);
    frontRight.moveWithVoltage(volts);
    backLeft.moveWithVoltage(volts);
    backRight.moveWithVoltage(volts);
  }

  // Should i move this to a different class? I feel like a teleop class wouldnt hurt...
  
}
