package frc.robot;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
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

    public void moveElevatorWithSaftey(double speed, boolean slowZones) { 
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

    // This code IS set and forget. NEVER run it once and leave..
    public void moveElevatorToPreset(String Preset) {
        double speed = 0;
        if (Preset == "L4") { setpointDouble = 1; } 
        else if (Preset == "L3") { setpointDouble = 0.5; } 
        else if (Preset == "L2") { setpointDouble = -1; } 
        else if (Preset == "L1") { setpointDouble = -1; }

        speed = setpointDouble *-1;

        moveElevatorWithSaftey(speed, true); 
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

    int grabSteps = 0;
    public void grabSequence(Joystick elevatorController) {
        var elevatorPose = elevatorMotor.getEncoder().getPosition();
        var armPose = armMotor.getEncoder().getPosition();
        if ((grabSteps == 0) && (elevatorPose < 5) && (armPose < -5)) {
            grabSteps = 1;
        } 
        else if (grabSteps == 1) {
            moveElevatorWithSaftey(-0.25, false);
            moveArmManual(0);
            if (elevatorPose > (slowZone+7.5)) { grabSteps = 2; }
        } 
        else if (grabSteps == 2) {
            moveElevatorWithSaftey(0, false);
            moveArmManual(0.35);
            if (armPose > -1) { grabSteps = 3;}
        }
        else if (grabSteps == 3) {
            moveElevatorWithSaftey(0.25, false);
            moveArmManual(0);
            if (elevatorPose < (0.25)) { grabSteps = 4;}
        } 
        else if (grabSteps == 4) {
            moveElevatorWithSaftey(0, true);
            moveArmManual(0);
            elevatorController.setRumble(RumbleType.kLeftRumble, 0.1);
        } else {
            moveArmManual(0);
            moveElevatorWithSaftey(0, true);
        }
    }
    


    public void elevatorLogic(Joystick elevatorController) {
        if (elevatorController.getRawButton(5)) {
            double flipperSpeed = (elevatorController.getRawAxis(2) - elevatorController.getRawAxis(3)); 
            moveElevatorWithSaftey(elevatorController.getRawAxis(5), true);
            moveFlipperManual(flipperSpeed);
            moveArmManual(elevatorController.getRawAxis(1));

        }  else if (elevatorController.getRawButton(4)) {
            moveElevatorToPreset("L4"); // L4
            // moveArmToPreset("L4");
        }  else if (elevatorController.getRawButton(2)) {
            moveElevatorToPreset("L3"); // L3
            // moveArmToPreset("L3");
        } else if (elevatorController.getRawButton(1)) {
            moveElevatorToPreset("L2"); // L2
            // moveArmToPreset("L2");
        } else if (elevatorController.getRawButton(6)) {
            grabSequence(elevatorController);
        }

        else {
            moveElevatorWithSaftey(0, true);
            moveFlipperManual(0);
            moveArmManual(0);
            grabSteps = 0;
        }

        

        reportPose();

    }
}
