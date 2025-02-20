package frc.robot;

import java.lang.StackWalker.Option;
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
    private Drivetrain drivetrain = Constants.swerveDrivetrain;
    private final Optional<Trajectory<SwerveSample>> trajectory = Choreo.loadTrajectory("myTrajectory");
    
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
        }
        timer.restart();
    }

    public void followTrajectory(SwerveSample sample) {
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

    public void autonomousPeriodic() {
        if (trajectory.isPresent()) {
            // Sample the trajectory at the current time into the autonomous period
            Optional<SwerveSample> sample = trajectory.get().sampleAt(timer.get(), isRedAlliance());

            if (sample.isPresent()) {
                followTrajectory(sample);
            }
        }
    }

}
