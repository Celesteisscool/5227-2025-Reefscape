package frc.robot;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;

public class Elevator {
    private SparkMax elevatorMotor = new SparkMax(25, SparkMax.MotorType.kBrushless);
    private PIDController elevatorPIDController = new PIDController(1, 0, 0);
    private XboxController elevatorController = Constants.elevatorController;

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

    // One liner elevator logic <3
    public void moveElevatorManual(double speed) { elevatorMotor.setVoltage(speed); }
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

    public void elevatorLogic(XboxController XboxController) {
        // Probably should switch to something more robust... Most likely something command based.
        // if (XboxController.getAButton()) { setPreset1(); }
        // else if (XboxController.getBButton()) { setPreset2(); }
        // else if (XboxController.getXButton()) { moveElevatorToPreset(1); }
        // else if (XboxController.getYButton()) { moveElevatorToPreset(2); }
        // else if (XboxController.getLeftBumperButton()) { moveElevatorManual(XboxController.getRightY()); }
        // else { moveElevatorManual(0); } // Prevents elevator from moving when not instructed.
        
        if (elevatorController.getAButton()) {
            double speed = elevatorController.getRightY() * 3;
            moveElevatorManual(speed);
        } else {
            moveElevatorManual(0);
        }

    }
}
