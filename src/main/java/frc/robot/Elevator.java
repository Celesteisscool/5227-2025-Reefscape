package frc.robot;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; 

public class Elevator {
    private SparkMax elevatorMotor = new SparkMax(25, SparkMax.MotorType.kBrushless);
    private SparkMax armMotor = new SparkMax(28, SparkMax.MotorType.kBrushless);
    private SparkMax flipperMotor = new SparkMax(27, SparkMax.MotorType.kBrushless);

    private Constraints elevatorConstraints = new Constraints(0.3, 0.3);
    private ProfiledPIDController elevatorPIDController = new ProfiledPIDController(0.1, 0, 0, elevatorConstraints);
    private ElevatorFeedforward elevatorFeedforward = new ElevatorFeedforward(0, 0, 0);
    private TrapezoidProfile trapezoidProfile = new TrapezoidProfile(elevatorConstraints);

    double preset1;
    double preset2;
    double setpointDouble;
    double setPID;
    double setFeedForward;

    double elevatorMaxHeight = 160;
    double elevatorMinHeight = 0.5;
    double elevatorSlowSpotHigh = elevatorMaxHeight-15;
    double elevatorSlowSpotLow = elevatorMinHeight+15;

    double flipperSetPose;
    double flipperPoseMin = -12.5;
    double flipperPoseMax = 0;

    TrapezoidProfile.State setpointState = new TrapezoidProfile.State(0, 0);
    TrapezoidProfile.State goalState = new TrapezoidProfile.State(0, 0);

    public Elevator() {
        elevatorMotor.getEncoder().setPosition(0);
        armMotor.getEncoder().setPosition(0);
        flipperMotor.getEncoder().setPosition(0);
    }

    public void moveElevatorManual(double speed, boolean driverMode) { 
        speed = Math.min(1, Math.max(speed, -1)); // Caps it just incase
        var elevatorPose = elevatorMotor.getEncoder().getPosition();
        if ((elevatorPose >= elevatorMaxHeight) && (speed < 0)) {
            speed = 0; // Stops if we are higher then max height
        } 
        if ((elevatorPose <= elevatorMinHeight) && (speed > 0)) {
            speed = 0; // Stops if we are lower then max height
        } 

        if ((elevatorPose > elevatorSlowSpotHigh) && driverMode) {
            speed *= 0.5;
        } else if ((elevatorPose < elevatorSlowSpotLow) && driverMode) {
            speed *= 0.5;
        }
        if (driverMode) { speed *= -4;}
        else {speed *= -1;}
        elevatorMotor.setVoltage(speed);
    }

    public void moveFlipperPoses(Joystick elevatorController) {
        var POV = elevatorController.getPOV();
        
        if (POV == 0) {
            flipperSetPose = flipperPoseMax + 10;
        } else if (POV == 180) {
            flipperSetPose = flipperPoseMin - 10;
        }

        double flipperPose = flipperMotor.getEncoder().getPosition();
        double speed = 0;
        if (flipperSetPose > flipperPose )  {
            speed = -1.0;
        } else if (flipperSetPose < flipperPose) {
            speed = 1.0;
        }
        moveFlipperManual(speed);
    }


    public void moveFlipperManual(double speed) { 
        
        var flipperPose = flipperMotor.getEncoder().getPosition();
        if ((flipperPose < flipperPoseMin) && speed > 0) {
            speed = 0;
        }
        if ((flipperPose > flipperPoseMax) && speed < 0) {
            speed = 0;
        }
        
        speed *= -2;
        flipperMotor.setVoltage(speed); 
        
    }

    public void moveArmManual(double speed) { armMotor.setVoltage(speed); }

    // This code IS set and forget. NEVER run it once and leave..
    public void moveElevatorToPreset(int Preset) {
        if (Preset == 0) { setpointDouble = 0.25; } 
        else if (Preset == 1) { setpointDouble = 0.75; }
        setpointDouble *= elevatorMaxHeight;

        elevatorPIDController.setGoal(setpointDouble);

        
        
        double elevatorPose = elevatorMotor.getEncoder().getPosition();
        setPID = (elevatorPIDController.calculate(elevatorPose));
        setFeedForward = elevatorFeedforward.calculate(elevatorMotor.getEncoder().getVelocity());
        moveElevatorManual((setPID + setFeedForward)*-1, false);

        SmartDashboard.putNumber("Elevator Pose", elevatorMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Desired Pose", elevatorPIDController.getGoal().position);
    }

    public void reportPose() {
        SmartDashboard.putNumber("Elevator Pose", elevatorMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Arm Pose", armMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Flipper Pose", flipperMotor.getEncoder().getPosition());
    }

    public void updatePIDandFF() {
        double elevatorP =  0;
        double elevatorI =  0;
        double elevatorD =  0;
        double elevatorG =  0.25; //Locked In
        double elevatorS =  0;
        double elevatorV =  0;
        double elevatorA =  0;
        double elevatorMV = 0.75;
        double elevatorMA = 0.75;

        elevatorPIDController.setP(elevatorP);
        elevatorPIDController.setI(elevatorI);
        elevatorPIDController.setD(elevatorD);
        elevatorPIDController.setConstraints(new Constraints(elevatorMV, elevatorMA));
        elevatorFeedforward = new ElevatorFeedforward(elevatorS, elevatorG, elevatorV, elevatorA);
    }


    public void elevatorLogic(Joystick elevatorController) {
        // Probably should switch to something more robust... Most likely something command based.
        // if (XboxController.getAButton()) { setPreset1(); }
        // else if (XboxController.getBButton()) { setPreset2(); }
        // else if (XboxController.getXButton()) { moveElevatorToPreset(1); }
        // else if (XboxController.getYButton()) { moveElevatorToPreset(2); }
        // else if (XboxController.getLeftBumperButton()) { moveElevatorManual(XboxController.getRightY()); }
        // else { moveElevatorManual(0); } // Prevents elevator from moving when not instructed.
        
        if (elevatorController.getRawButton(1)) {
            double armSpeed = elevatorController.getRawAxis(1) * 2;
            double flipperSpeed = (elevatorController.getRawAxis(2) - elevatorController.getRawAxis(3)); 
            moveElevatorManual(elevatorController.getRawAxis(5), true);
            moveArmManual(armSpeed);
            moveFlipperManual(flipperSpeed);
        }  else if (elevatorController.getRawButton(2)) {
            moveElevatorToPreset(0);
        }  else if (elevatorController.getRawButton(3)) {
            moveElevatorToPreset(1);
        }

        else {
            moveElevatorManual(0, true);
            moveArmManual(0);
            moveFlipperManual(0);
        }

        if (elevatorController.getRawButton(4)) {
            updatePIDandFF();
        }

        reportPose();

    }
}
