package frc.robot;

import java.util.Optional;

import javax.xml.xpath.XPath;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;

public class Auto {
    private PIDController xController   = Constants.xController;
    private PIDController yController   = Constants.yController;
    private PIDController rotController = Constants.rotController;
    private Drivetrain drivetrain = Constants.drivetrainClass;
    private final Optional<Trajectory<SwerveSample>> trajectory = Choreo.loadTrajectory("testAuto");
    public final SendableChooser<String> autoChooser = new SendableChooser<>();
    private final Timer timer = new Timer();
    public Auto() {
        rotController.enableContinuousInput(-Math.PI, Math.PI);
    }

    private boolean isRedAlliance() {
        return DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red);
    }

    public void setupDash() {
        autoChooser.addOption("backup", "backup");
        autoChooser.addOption("l4", "l4");
        autoChooser.setDefaultOption("l4", "l4");
        SmartDashboard.putData(autoChooser);
    }

    public void autonomousInitChoreo() {
        if (trajectory.isPresent()) {
            Optional<Pose2d> initialPose = trajectory.get().getInitialPose(isRedAlliance());

            if (initialPose.isPresent()) {
                drivetrain.resetOdometry(initialPose.get());
            }
        }
        timer.restart();
    }

    public void followTrajectoryChoreo(Optional<SwerveSample> inputSample) {
        if (inputSample.isPresent()) {
            var sample = inputSample.get();
            // Get the current pose of the robot
            Pose2d pose = drivetrain.odometry.getPoseMeters();

            // Generate the next speeds for the robot
            ChassisSpeeds speeds = new ChassisSpeeds(
                sample.vx + xController.calculate(pose.getX(), sample.x),
                sample.vy + yController.calculate(pose.getY(), sample.y),
                sample.omega + rotController.calculate(pose.getRotation().getRadians(), sample.heading)
            );

            // Apply the generated speeds
            drivetrain.driveFieldRelativeFromSpeed(speeds);
        }   
    }

    public void autonomousPeriodicChoreo() {
        if (trajectory.isPresent()) {
            // Sample the trajectory at the current time into the autonomous period
            Optional<SwerveSample> sample = trajectory.get().sampleAt(timer.get(), isRedAlliance());

            if (sample.isPresent()) {
                followTrajectoryChoreo(sample);
            }
        }
    }



    public void autoInit() {
        timer.reset();
        timer.start();
    }

    public void runSelectedAuto() {
        if (autoChooser.getSelected() == "l4") {
            l4Auto();
        } else {
            backupAuto();
        }
    }

    public void l4Auto() {
        double xSpeed = 0;
        double ySpeed = 0;
        double rotSpeed = 0;
        if (timer.get() < 2.8) {
            xSpeed = (0.1 * Constants.maxSwerveSpeed * 0.95 * 1);
		    ySpeed = ((0) * Constants.maxSwerveSpeed * 0.95 * 1);
		    rotSpeed = ((0) * Constants.maxSwerveAngularSpeed * 0.95 * 1);
            Constants.elevatorClass.moveL4();
            
        } else if (timer.get() < 5) {
            xSpeed = (0 * Constants.maxSwerveSpeed * 0.95 * 1);
		    ySpeed = ((0) * Constants.maxSwerveSpeed * 0.95 * 1);
		    rotSpeed = ((0) * Constants.maxSwerveAngularSpeed * 0.95 * 1);
            Constants.elevatorClass.moveL4();
        } else if (timer.get() < 6) {
            Constants.elevatorClass.moveArmManual(0.5);
        }

        else {
            xSpeed = 0;
            ySpeed = 0;
            rotSpeed = 0;
        }

        drivetrain.drive(
			xSpeed, 
			ySpeed, 
			rotSpeed, 
			true
		);
    }

    public void backupAuto() {
        double xSpeed = 0;
        double ySpeed = 0;
        double rotSpeed = 0;
        if (timer.get() < 1.75) {
            xSpeed = (0.25 * Constants.maxSwerveSpeed * 0.95 * 1);
		    ySpeed = ((0) * Constants.maxSwerveSpeed * 0.95 * 1);
		    rotSpeed = ((0) * Constants.maxSwerveAngularSpeed * 0.95 * 1);
            Constants.elevatorClass.moveL4();
            
        } else {
            xSpeed = 0;
            ySpeed = 0;
            rotSpeed = 0;
        }

        drivetrain.drive(
			xSpeed, 
			ySpeed, 
			rotSpeed, 
			true
		);
    }

}