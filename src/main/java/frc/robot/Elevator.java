package frc.robot;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; 

public class Elevator {
    private SparkMax elevatorMotor = new SparkMax(25, SparkMax.MotorType.kBrushless);
    private SparkMax flipperMotor = new SparkMax(27, SparkMax.MotorType.kBrushless);
    
    private SparkMax armMotor = new SparkMax(28, SparkMax.MotorType.kBrushless);
    private SparkClosedLoopController armMotorController = armMotor.getClosedLoopController();

    double preset1;
    double preset2;
    double setpointDouble;
    double presetMax;

    double elevatorMaxHeight = 160;
    double elevatorMinHeight = 0.5;
    double slowZone = 15;

    double flipperSetPose;
    double flipperPoseMin = -12.5;
    double flipperPoseMax = 0;

    double minArmPose = -30;
    double maxArmPose = -0.5;

    public Elevator() { // Zeros Everything
        elevatorMotor.getEncoder().setPosition(0);
        armMotor.getEncoder().setPosition(0);
        flipperMotor.getEncoder().setPosition(0);
    }

    public void moveElevatorWithSafety(double speed, boolean slowZones) { 
        speed = Math.min(1, Math.max(speed, -1)); // Caps it just incase
        speed *= -1;
        
        var elevatorPose = elevatorMotor.getEncoder().getPosition();

        if ((elevatorPose >= elevatorMaxHeight) && (speed > 0)) {
            speed = 0; // Stops if we are higher then max height
        } 
        if ((elevatorPose <= elevatorMinHeight) && (speed < 0)) {
            speed = 0; // Stops if we are lower then max height
        } 

        if (elevatorPose > (elevatorMaxHeight - slowZone) && slowZones) {
            speed *= 0.5; 
        } else if (elevatorPose < (elevatorMinHeight + slowZone) && slowZones) {
            speed *= 0.5; 
        }        
        speed *= 4;
        elevatorMotor.setVoltage(speed);
    }

    private void moveArmManual(double speed) {
        double armPose = (armMotor.getEncoder().getPosition());
        
        if ((speed < 0) && (armPose < minArmPose)) {
            speed = 0;
        }
        if ((speed > 0) && (armPose > maxArmPose)) {
            speed = 0;
        }
        armMotor.setVoltage(speed * 2);
    }


    public void moveArmToPreset(String Preset) {
        double armPose = armMotor.getEncoder().getPosition();
        

        if (Preset == "L4") { setpointDouble = 1; }
        setpointDouble *= minArmPose;

        armMotorController.setReference(armPose, ControlType.kPosition);
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


    

    public void reportPose() {
        SmartDashboard.putNumber("Elevator Pose", elevatorMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Arm Pose", armMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Flipper Pose", flipperMotor.getEncoder().getPosition());
    }



    int L4Steps = 0; // THANK YOU ZEKE YOU ARE THE BEST <3

    public void grabSequence() {
        var elevatorPose = elevatorMotor.getEncoder().getPosition();
        var armPose = armMotor.getEncoder().getPosition();
        double elevatorPresetSpeed = 0.25; // increase this a bit its really slow rn
        double armPresetSpeed = 0.35;

        switch (L4Steps) {
            case 0:
                if ((elevatorPose < 5) && (armPose < -5)) { L4Steps = 1; }
                break;
            case 1:
                moveElevatorWithSafety(-elevatorPresetSpeed, false);
                moveArmManual(0);
                if (elevatorPose > (slowZone + 7.5)) { L4Steps = 2; }
                break;
            case 2:
                moveElevatorWithSafety(0, false);
                moveArmManual(armPresetSpeed);
                if (armPose > -1) { L4Steps = 3; }
                break;
            case 3:
                moveElevatorWithSafety(elevatorPresetSpeed, false);
                moveArmManual(0);
                if (elevatorPose < 0.25) { L4Steps = 4; }
                break;
            case 4:
                moveElevatorWithSafety(0, false);
                moveArmManual(0);
                break;
            default:
                moveArmManual(0);
                moveElevatorWithSafety(0, false);
                break;
        }
    }

    public void moveL4() {
        var elevatorPose = elevatorMotor.getEncoder().getPosition();
        var armPose = armMotor.getEncoder().getPosition();
        double elevatorPresetSpeed = 0.25; // bump this once we know it works
        double armPresetSpeed = 0.35;

        double L4ElevatorPose = elevatorMaxHeight - 1;
        double L4ArmPose = minArmPose;

        switch (L4Steps) {
            case 0:
                if ((elevatorPose < 5) && (armPose > -1)) { L4Steps = 1; } // Clamped on piece
                break;
            case 1:
                moveElevatorWithSafety(-elevatorPresetSpeed, true); // Moves upward
                moveArmManual(0);
                if (elevatorPose > L4ElevatorPose) { L4Steps = 2; }
                break;
            case 2:
                moveElevatorWithSafety(0, false);
                moveArmManual(-armPresetSpeed); // Sends arm outward
                if (armPose < L4ArmPose) { L4Steps = 3; }
                break;
            case 3:
                moveElevatorWithSafety(0, false);
                moveArmManual(0);
                break;
            default:
                moveArmManual(0);
                moveElevatorWithSafety(0, false);
                break;
        }
    }



    public void elevatorLogic(Joystick elevatorController) {
        if (elevatorController.getRawButton(5)) {
            double flipperSpeed = (elevatorController.getRawAxis(2) - elevatorController.getRawAxis(3)); 
            moveElevatorWithSafety(elevatorController.getRawAxis(5), true);
            moveFlipperManual(flipperSpeed);
            moveArmManual(elevatorController.getRawAxis(1));

        }  else if (elevatorController.getRawButton(4)) { // Y
            moveL4();
        }  else if (elevatorController.getRawButton(2)) { //B
            // L3
        } else if (elevatorController.getRawButton(1)) { // A 
            // L2
        } else if (elevatorController.getRawButton(6)) { // RB (Change to X)
            grabSequence();
        }

        else {
            moveElevatorWithSafety(0, true);
            moveFlipperManual(0);
            moveArmManual(0);
            L4Steps = 0;
        }

        

        reportPose();

    }
}
