package frc.robot;

import java.util.Optional;
import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;

public class Auto {
    private PIDController xController   = Constants.xController;
    private PIDController yController   = Constants.yController;
    private PIDController rotController = Constants.rotController;
    private Drivetrain drivetrain = Constants.drivetrainClass;
    private final Optional<Trajectory<SwerveSample>> trajectory = Choreo.loadTrajectory("testAuto");
    
    private final Timer timer = new Timer();
    public Auto() {
        rotController.enableContinuousInput(-Math.PI, Math.PI);
    }

    private boolean isRedAlliance() {
        return DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red);
    }

    public void autonomousInit() {
        if (trajectory.isPresent()) {
            Optional<Pose2d> initialPose = trajectory.get().getInitialPose(isRedAlliance());

            if (initialPose.isPresent()) {
                drivetrain.resetOdometry(initialPose.get());
            }
        }
        timer.restart();
    }

    public void followTrajectory(Optional<SwerveSample> inputSample) {
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

    public void autonomousPeriodic() {
        if (trajectory.isPresent()) {
            // Sample the trajectory at the current time into the autonomous period
            Optional<SwerveSample> sample = trajectory.get().sampleAt(timer.get(), isRedAlliance());

            if (sample.isPresent()) {
                followTrajectory(sample);
            }
        }
    }

    public void tempAutoCodeInit() {
        timer.reset();
        timer.start();
    }

    public void tempAutoCodePeriodic() {
        double xSpeed;
        double ySpeed;
        double rotSpeed;
        if (timer.get() < 1.75) {
            xSpeed = (0.25 * Constants.maxSwerveSpeed * 0.95 * 1);
		    ySpeed = ((0) * Constants.maxSwerveSpeed * 0.95 * 1);
		    rotSpeed = ((0) * Constants.maxSwerveAngularSpeed * 0.95 * 1);
            
        } else {
            xSpeed = (0 * Constants.maxSwerveSpeed * 0.95 * 1);
		    ySpeed = ((0) * Constants.maxSwerveSpeed * 0.95 * 1);
		    rotSpeed = ((0) * Constants.maxSwerveAngularSpeed * 0.95 * 1);
        }

        drivetrain.drive(
			xSpeed, 
			ySpeed, 
			rotSpeed, 
			true
		);
    }

}