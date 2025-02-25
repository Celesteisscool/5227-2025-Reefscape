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

    public Elevator() {
        elevatorMotor.getEncoder().setPosition(0);
        armMotor.getEncoder().setPosition(0);
    }

    // One liner elevator logic <3
    public void moveElevatorManual(double speed) { elevatorMotor.setVoltage(speed); }
    public void moveArmManual(double speed) { armMotor.setVoltage(speed); }
    public void moveFlipperManual(double speed) { flipperMotor.setVoltage(speed); }
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
            double elevatorSpeed = elevatorController.getRawAxis(5) * -4;
            double armSpeed = elevatorController.getRawAxis(1) * -2;
            double flipperSpeed = (elevatorController.getRawAxis(2) - elevatorController.getRawAxis(3) )* -2; 
            moveElevatorManual(elevatorSpeed);
            moveArmManual(armSpeed);
            moveFlipperManual(flipperSpeed);
        } else {
            moveElevatorManual(0);
            moveArmManual(0);
            moveFlipperManual(0);
        }

        reportPose();

    }
}
