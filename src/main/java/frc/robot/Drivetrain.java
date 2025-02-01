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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain extends SubsystemBase {
  
  public static final double maxSwerveSpeed = 1.5; // 3 meters per second
  public static final double maxSwerveAngularSpeed = Math.PI; // 1/2 rotation per second

  private final Translation2d frontLeftLocation = new Translation2d(0.381, 0.381);
  private final Translation2d frontRightLocation = new Translation2d(0.381, -0.381);
  private final Translation2d backLeftLocation = new Translation2d(-0.381, 0.381);
  private final Translation2d backRightLocation = new Translation2d(-0.381, -0.381);

  private final SwerveModule frontLeft = new SwerveModule(5, 6, 11);
  private final SwerveModule frontRight = new SwerveModule(3,4,10);
  private final SwerveModule backLeft = new SwerveModule(7,8,12);
  private final SwerveModule backRight = new SwerveModule(1,2,9);

  private SlewRateLimiter slewRateLimiterX = new SlewRateLimiter(2);
  private SlewRateLimiter slewRateLimiterY = new SlewRateLimiter(2);
  private SlewRateLimiter slewRateLimiterRot = new SlewRateLimiter(3);

  private final PPHolonomicDriveController pathplannerDriveController = new PPHolonomicDriveController(
        new PIDConstants(5.0, 0.0, 0.0), // DEFINITELY gotta be changed.
        new PIDConstants(5.0, 0.0, 0.0)
  );

  private final Pigeon2 pigeon2 = new Pigeon2(32);

  StructArrayPublisher<SwerveModuleState> DesiredState = NetworkTableInstance.getDefault().getStructArrayTopic("Desired State", SwerveModuleState.struct).publish();
  StructArrayPublisher<SwerveModuleState> CurrentState = NetworkTableInstance.getDefault().getStructArrayTopic("Current State", SwerveModuleState.struct).publish();

  private final SendableChooser<Command> autoChooser;

  private final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(
          frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

  private final SwerveDriveOdometry odometry =
      new SwerveDriveOdometry(
          kinematics,
          pigeon2.getRotation2d(),
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
  private void resetPose(Pose2d pose) { odometry.resetPosition(pigeon2.getRotation2d(), returnWheelPositions(), pose); }
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
    var swerveModuleStates = kinematics.toSwerveModuleStates(ChassisSpeeds.discretize(fieldRelative
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, pigeon2.getRotation2d()) // Felid relative
                    : new ChassisSpeeds(xSpeed, ySpeed, rot), // Absolute
                periodSeconds));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, maxSwerveSpeed);
    setStates(swerveModuleStates);
    updateDashboard();
  }

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
        pigeon2.getRotation2d(),
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

  public void driveWithJoystick(boolean fieldRelative, XboxController driverController, Drivetrain swerveDrive, double period) {
    var speedLimit = 1 - (driverController.getRightTriggerAxis() * .5);
    var xSpeed = (driverController.getLeftY()* Drivetrain.maxSwerveSpeed * speedLimit);
    var ySpeed = (driverController.getLeftX() * Drivetrain.maxSwerveSpeed * speedLimit);
    var rotSpeed = (driverController.getRightX() * Drivetrain.maxSwerveAngularSpeed * speedLimit) * -1; //invert so that turning is more natural

    var POV = driverController.getPOV();
    if (POV != -1) { //Drives with the DPad instead of the joystick for perfect 45° angles
      var POVRadians = Math.toRadians(POV);
      xSpeed = Math.cos(POVRadians) * 0.25;
      ySpeed = Math.sin(POVRadians) * 0.25;
      fieldRelative = false; // Forces it to be robot relative
    }
    
    swerveDrive.drive(
      slewRateLimiterX.calculate(xSpeed), 
      slewRateLimiterY.calculate(ySpeed), 
      slewRateLimiterRot.calculate(rotSpeed), 
      fieldRelative, period
    );
  }
}
