package frc.robot;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; 

public class Elevator {
    private SparkMax elevatorMotor = new SparkMax(25, SparkMax.MotorType.kBrushless);
    private SparkMax armMotor = new SparkMax(28, SparkMax.MotorType.kBrushless);
    private SparkMax flipperMotor = new SparkMax(27, SparkMax.MotorType.kBrushless);
    private PIDController elevatorPIDController = new PIDController(1, 0, 0);

    private double ffS = 0; //PLEASE tune these once the elevator is assembled!!!
    private double ffG = 0;
    private double ffV = 0;
    private double ffA = 0;
    private ElevatorFeedforward m_ElevatorFeedforward = new ElevatorFeedforward(ffS, ffG, ffV, ffA);

    double preset1;
    double preset2;
    double setpoint;
    double setPID;
    double setFeedForward;

    double elevatorMaxHeight = 160;
    double elevatorMinHeight = 0.5;
    double elevatorSlowSpotHigh = elevatorMaxHeight-15;
    double elevatorSlowSpotLow = elevatorMinHeight+15;

    double flipperSetPose;
    double flipperPoseMin = -12.5;
    double flipperPoseMax = 0;

    public Elevator() {
        elevatorMotor.getEncoder().setPosition(0);
        armMotor.getEncoder().setPosition(0);
        flipperMotor.getEncoder().setPosition(0);
    }

    public void moveElevatorManual(double speed) { 
        speed = Math.min(1, Math.max(speed, -1)); // Caps it just incase
        var elevatorPose = elevatorMotor.getEncoder().getPosition();
        if ((elevatorPose >= elevatorMaxHeight) && (speed < 0)) {
            speed = 0; // Stops if we are higher then max height
        } 
        if ((elevatorPose <= elevatorMinHeight) && (speed > 0)) {
            speed = 0; // Stops if we are lower then max height
        } 

        if (elevatorPose > elevatorSlowSpotHigh) {
            speed *= 0.5;
        } else if (elevatorPose < elevatorSlowSpotLow) {
            speed *= 0.5;
        }
        elevatorMotor.setVoltage(speed * -4);
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
    public void setPreset1() { preset1 = elevatorMotor.getEncoder().getPosition(); }
    public void setPreset2() { preset2 = elevatorMotor.getEncoder().getPosition(); }

    // This code IS set and forget. NEVER run it once and leave..
    public void moveElevatorToPreset(int Preset) {
        if (Preset == 1) { setpoint = preset1; } 
        else if (Preset == 2) { setpoint = preset2; }
        elevatorPIDController.setSetpoint(setpoint);
        
        setPID = (elevatorPIDController.calculate(elevatorMotor.getEncoder().getPosition(), setpoint));
        setFeedForward = m_ElevatorFeedforward.calculate(elevatorPIDController.getSetpoint());
        elevatorMotor.setVoltage(setPID + setFeedForward);
    }

    public void reportPose() {
        SmartDashboard.putNumber("Elevator Pose", elevatorMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Arm Pose", armMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Flipper Pose", flipperMotor.getEncoder().getPosition());
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
            double armSpeed = elevatorController.getRawAxis(1) * -2;
            double flipperSpeed = (elevatorController.getRawAxis(2) - elevatorController.getRawAxis(3)); 
            moveElevatorManual(elevatorController.getRawAxis(5));
            moveArmManual(armSpeed);
            moveFlipperManual(flipperSpeed);
        } else if (elevatorController.getRawButton(2)) {
            moveFlipperPoses(elevatorController);
        } else {
            moveElevatorManual(0);
            moveArmManual(0);
            moveFlipperManual(0);
        }

        reportPose();

    }
}
