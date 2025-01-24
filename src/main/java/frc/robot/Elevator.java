package frc.robot;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

public class Elevator {
    private SparkMax m_motor = new SparkMax(23, SparkMax.MotorType.kBrushless);
    private PIDController m_leftPidController = new PIDController(0.1, 0, 0);

    private double kS = 0;
    private double kG = 0;
    private double kV = 0;
    private double kA = 0;
    private ElevatorFeedforward m_ElevatorFeedforward = new ElevatorFeedforward(kS, kG, kV, kA);

    double preset1;
    double preset2;
    double setpoint;
    double setPID;
    double setFeedForward;

    public Elevator() {
        //m_motorRight.configure(SparkBaseConfig, null, null)
        //m_motorRight.setInverted(true);
    }

    public void moveElevatorManual(double speed) {
        m_motor.set(speed);
    }

    public void setPreset1() {
        preset1 = m_motor.getEncoder().getPosition();
    }

    public void setPreset2() {
        preset2 = m_motor.getEncoder().getPosition();
    }

    public void moveElevatorPreset(int Preset) {
        if (Preset == 1) {
            setpoint = preset1;
        } else if (Preset == 2) {
            setpoint = preset2;
        }
        m_leftPidController.setSetpoint(setpoint);
        
        setPID = (m_leftPidController.calculate(m_motor.getEncoder().getPosition(), setpoint));
        setFeedForward = m_ElevatorFeedforward.calculate(m_leftPidController.getSetpoint());
        m_motor.setVoltage(setPID + setFeedForward);
    }

    public void elevatorLogic(XboxController XboxController) {
        if (XboxController.getAButton()) {
            setPreset1();
        }
        if (XboxController.getBButton()) {
            setPreset2();
        }
        if (XboxController.getXButton()) {
            moveElevatorPreset(1);
        }
        if (XboxController.getYButton()) {
            moveElevatorPreset(2);
        }

        if (XboxController.getLeftBumperButton()) {
            moveElevatorManual(XboxController.getRightY());
        }
    }
}
