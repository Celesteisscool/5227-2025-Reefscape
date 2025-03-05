package frc.robot;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; 

public class Elevator {
    private SparkMax elevatorMotor = new SparkMax(25, SparkMax.MotorType.kBrushless);
    private SparkMax flipperMotor = new SparkMax(27, SparkMax.MotorType.kBrushless);
    
    private SparkMax armMotor = new SparkMax(28, SparkMax.MotorType.kBrushless);
    private SparkClosedLoopController armMotorController = armMotor.getClosedLoopController();

    XboxController elevatorController;

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
    double maxArmPose = -0.25;

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
        speed = Math.min(1, Math.max(speed, -1)); // Caps it just incase
        
        if ((speed < 0) && (armPose < minArmPose)) {
            speed = 0;
        }
        if ((speed > 0) && (armPose > maxArmPose)) {
            speed = 0;
        }
        armMotor.setVoltage(speed * 2);
    }

    public void moveFlipperPoses(int POV) {
        
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

    int grabSteps = 0; // THANK YOU ZEKE YOU ARE THE BEST <3
    public void grabSequence() {
        var elevatorPose = elevatorMotor.getEncoder().getPosition();
        var armPose = armMotor.getEncoder().getPosition();
        double elevatorPresetSpeedUp = 0.55; // increase this a bit its really slow rn
        double elevatorPresetSpeedDown = 0.45; // increase this a bit its really slow rn
        double armPresetSpeed = 0.45;

        switch (grabSteps) {
            case 0:
                if ((elevatorPose < 5) && (armPose < -5)) { grabSteps = 1; }
                break;
            case 1:
                moveElevatorWithSafety(-elevatorPresetSpeedUp, false);
                moveArmManual(0);
                if (elevatorPose > (slowZone + 7.5)) { grabSteps = 2; }
                break;
            case 2:
                moveElevatorWithSafety(0, false);
                moveArmManual(armPresetSpeed);
                if (armPose > maxArmPose) { grabSteps = 3; }
                break;
            case 3:
                moveElevatorWithSafety(elevatorPresetSpeedDown, false);
                moveArmManual(0);
                if (elevatorPose < 0.25) { grabSteps = 4; }
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

    int L4Steps = 0; // THANK YOU ZEKE YOU ARE THE BEST <3
    public void moveL4() {
        var elevatorPose = elevatorMotor.getEncoder().getPosition();
        var armPose = armMotor.getEncoder().getPosition();
        double elevatorPresetSpeed = 0.75; // bump this once we know it works
        double armPresetSpeed = 0.45;

        double L4ElevatorPose = elevatorMaxHeight - 2;
        double L4MoveArm = L4ElevatorPose / 2;
        double L4ArmPose = minArmPose + 10;

        boolean L4armDone = (armPose < L4ArmPose);
        boolean L4elevatorDone = (elevatorPose > L4ElevatorPose);

        switch (L4Steps) {
            case 0:
                if ((elevatorPose < 5) && (armPose > -1)) { L4Steps = 1; } // Checks if we're clamped on piece
                break;
            case 1:
                moveElevatorWithSafety(-elevatorPresetSpeed, false); // Moves elevator upward 
                moveArmManual(0);                                        // No slow because we're moving up
                if (elevatorPose > L4MoveArm) { L4Steps = 2; } // Moves only halfway
                break;
            case 2:
                moveElevatorWithSafety(-elevatorPresetSpeed, true); // Moves elevator up
                moveArmManual(-armPresetSpeed); // And arm up

                // Spliting paths 0.0
                if (L4armDone)      { L4Steps = 3; } // Checks if our arm is done first
                if (L4elevatorDone) { L4Steps = 4; } // Checks if elevator is done first
                break;

            case 3: //Stops arm
                moveElevatorWithSafety(-elevatorPresetSpeed, true); 
                moveArmManual(0);
                if (L4elevatorDone) { L4Steps = 5; } // Stops all
                break;
            case 4: //Stops elevator
                moveElevatorWithSafety(0, true);
                moveArmManual(-armPresetSpeed);
                if (L4armDone) { L4Steps = 5; } //Stops All
                break;

            // Paths merge now (●'◡'●)
            case 5: //Stops all
                moveElevatorWithSafety(0, false);
                moveArmManual(0);
                break; 
            default:
                moveArmManual(0);
                moveElevatorWithSafety(0, false);
                break;
        }
    }

    int L3Steps = 0; // THANK YOU ZEKE YOU ARE THE BEST <3
    public void moveL3() {
        var elevatorPose = elevatorMotor.getEncoder().getPosition();
        var armPose = armMotor.getEncoder().getPosition();
        double elevatorPresetSpeed = 0.75; // bump this once we know it works
        double armPresetSpeed = 0.45;

        double L3ElevatorPose = 85;
        double L3MoveArm = L3ElevatorPose / 2;
        double L3ArmPose = -22.5;

        boolean L3armDone = (armPose < L3ArmPose);
        boolean L3elevatorDone = (elevatorPose > L3ElevatorPose);

        switch (L3Steps) {
            case 0:
                if ((elevatorPose < 5) && (armPose > -1)) { L3Steps = 1; } // Checks if we're clamped on piece
                break;
            case 1:
                moveElevatorWithSafety(-elevatorPresetSpeed, false); // Moves elevator upward 
                moveArmManual(0);                                        // No slow because we're moving up
                if (elevatorPose > L3MoveArm) { L3Steps = 2; } // Moves only halfway
                break;
            case 2:
                moveElevatorWithSafety(-elevatorPresetSpeed, true); // Moves elevator up
                moveArmManual(-armPresetSpeed); // And arm up

                // Spliting paths 0.0
                if (L3armDone)      { L3Steps = 3; } // Checks if our arm is done first
                if (L3elevatorDone) { L3Steps = 4; } // Checks if elevator is done first
                break;

            case 3: //Stops arm
                moveElevatorWithSafety(-elevatorPresetSpeed, true); 
                moveArmManual(0);
                if (L3elevatorDone) { L3Steps = 5; } // Stops all
                break;
            case 4: //Stops elevator
                moveElevatorWithSafety(0, true);
                moveArmManual(-armPresetSpeed);
                if (L3armDone) { L3Steps = 5; } //Stops All
                break;

            // Paths merge now (●'◡'●)
            case 5: //Stops all
                moveElevatorWithSafety(0, false);
                moveArmManual(0);
                break; 
            default:
                moveArmManual(0);
                moveElevatorWithSafety(0, false);
                break;
        }
    }

    int L2Steps = 0;
    public void moveL2() {
        var elevatorPose = elevatorMotor.getEncoder().getPosition();
        var armPose = armMotor.getEncoder().getPosition();
        double elevatorPresetSpeed = 0.6; // bump this once we know it works
        double armPresetSpeed = 0.65;

        double L2ElevatorPose = slowZone + 15;
        double L2ArmPose = -22.5;

        boolean L2armDone = (armPose < L2ArmPose);
        boolean L2elevatorUp = (elevatorPose > L2ElevatorPose);
        boolean L2elevatorDown = (elevatorPose < 0.25);

        switch (L2Steps) {
            case 0:
                if ((elevatorPose < 5) && (armPose > -1)) { L2Steps = 1; } // Checks if we're clamped on piece
                break;

            case 1:
                moveElevatorWithSafety(-elevatorPresetSpeed, false); // Moves elevator upward 
                moveArmManual(0);                                        // No slow because we're moving up
                if (L2elevatorUp) { L2Steps = 2; } // If we can move arm yet
                break;

            case 2: // Arm Up
                moveElevatorWithSafety(0, true); // Stop
                moveArmManual(-armPresetSpeed); // Move arm
                if (L2armDone) { L2Steps = 3; } // Checks if our arm is done first
                break;

            case 3: // Elevator down
                moveElevatorWithSafety(elevatorPresetSpeed, true); 
                moveArmManual(0);
                if (L2elevatorDown) { L2Steps = 4; } // Stops all
                break;

            case 4: //Stops all
                moveElevatorWithSafety(0, false);
                moveArmManual(0);
                break; 
            default:
                moveArmManual(0);
                moveElevatorWithSafety(0, false);
                break;
        }
    }

    public void elevatorLogic() {
        elevatorController = Constants.elevatorController;
        if (elevatorController.getLeftBumperButton()) {
            double flipperSpeed = (elevatorController.getLeftTriggerAxis() - elevatorController.getRightTriggerAxis()); 
            moveElevatorWithSafety(elevatorController.getRightY(), true);
            if (elevatorController.getPOV() != -1) {
                moveFlipperPoses(elevatorController.getPOV());
            } else {
                moveFlipperManual(flipperSpeed);
            }
            moveArmManual(elevatorController.getLeftY());

        }  else if (elevatorController.getYButton()) { 
            moveL4();
        }  else if (elevatorController.getBButton()) {
            moveL3();
        } else if (elevatorController.getAButton()) { 
            moveL2();
        } else if (elevatorController.getXButton()) { 
            grabSequence();
        }

        else {
            moveElevatorWithSafety(0, true);
            moveFlipperManual(0);
            moveArmManual(0);
            L4Steps = 0;
            L3Steps = 0;
            L2Steps = 0;
            grabSteps = 0;
        }

        reportPose();

    }

    public double getElevatorSlowdown() {
        double elevatorPose = elevatorMotor.getEncoder().getPosition();
        double output = 1;
        if (elevatorPose > (elevatorMaxHeight / 2)) {
            var slowdownPose = (elevatorPose - (elevatorMaxHeight / 2));
            var slowdownSpeed = 1 / ((slowdownPose));
            output *= slowdownSpeed;
        } 
        return output;
    }
}
