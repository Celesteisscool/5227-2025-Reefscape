package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;

public class Constants {
    // Drivetrain Constants
    public static final double maxSwerveSpeed = 1.5; // 1.5 meters per second
    public static final double maxSwerveAngularSpeed = Math.PI; // 1/2 rotation per second

    public static final int[] frontLeftIDs = {5, 6, 11};
    public static final int[] frontRightIDs = {3, 4, 10};
    public static final int[] backLeftIDs = {7, 8, 12};
    public static final int[] backRightIDs = {1, 2, 9};
    public static final int gyroID = 32;

    public static final SlewRateLimiter slewRateLimiterX = new SlewRateLimiter(2.5);
    public static final SlewRateLimiter slewRateLimiterY = new SlewRateLimiter(2.5);
    public static final SlewRateLimiter slewRateLimiterRot = new SlewRateLimiter(3);

    // SwerveModule Constants
    public static final Translation2d frontLeftLocation = new Translation2d(0.381, 0.381);
    public static final Translation2d frontRightLocation = new Translation2d(0.381, -0.381);
    public static final Translation2d backLeftLocation = new Translation2d(-0.381, 0.381);
    public static final Translation2d backRightLocation = new Translation2d(-0.381, -0.381);

    // Global Constants 
    public static Drivetrain swerveDrivetrain = new Drivetrain();
    public static Elevator elevatorClass = new Elevator();

    public static XboxController driverController = new XboxController(0);
    public static XboxController elevatorController = new XboxController(1);
    
    // Vision Constants
    public static final Vision vision = new Vision();

    //Auto Constants 
    public final static PIDController xController =   new PIDController(10.0, 0.0, 0.0);
    public final static PIDController yController =   new PIDController(10.0, 0.0, 0.0);
    public final static PIDController rotController = new PIDController(7.5, 0.0, 0.0); 

}
