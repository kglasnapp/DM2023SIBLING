package frc.robot.subsystems;

import static frc.robot.Util.logf;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.ShowPID;
import frc.robot.subsystems.LedSubsystem.Leds;

/**
 * REV Smart Motion Guide
 * 
 * The SPARK MAX includes a control mode, REV Smart Motion which is used to 
 * control the position of the motor, and includes a max velocity and max 
 * acceleration parameter to ensure the motor moves in a smooth and predictable 
 * way. This is done by generating a motion profile on the fly in SPARK MAX and 
 * controlling the velocity of the motor to follow this profile.
 * 
 * Since REV Smart Motion uses the velocity to track a profile, there are only 
 * two steps required to configure this mode:
 *    1) Tune a velocity PID loop for the mechanism
 *    2) Configure the smart motion parameters
 * 
 * Tuning the Velocity PID Loop
 * 
 * The most important part of tuning any closed loop control such as the velocity 
 * PID, is to graph the inputs and outputs to understand exactly what is happening. 
 * For tuning the Velocity PID loop, at a minimum we recommend graphing:
 *
 *    1) The velocity of the mechanism (‘Process variable’)
 *    2) The commanded velocity value (‘Setpoint’)
 *    3) The applied output
 *
 * This example will use ShuffleBoard to graph the above parameters. Make sure to
 * load the shuffleboard.json file in the root of this directory to get the full
 * effect of the GUI layout.
 */

public class ElevatorSubsystem extends SubsystemBase {
    private static final int Elevator_MOTOR_ID = 11;
    private double lastElevatorPosition = 0;
    private double lastElevatorSetPoint = 0;
    private SparkMaxLimitSwitch forwardLimit;
    private SparkMaxLimitSwitch reverseLimit;
    private CANSparkMax elevatorMotor;
    private SparkMaxPIDController pidController;
    private RelativeEncoder tiltEncoder;
    private PID_MAX pid = new PID_MAX();
    private double elevatorTicksPerInch = 1000;

    public static double HOME_POS = 0.0;
    public static double LOW_POS = 0.0;
    public static double MEDIUM_POS = 0.0;
    public static double HIGH_POS = 0.0;

    public ElevatorSubsystem() {

        // Setup paramters for the tilt motor
        elevatorMotor = new CANSparkMax(Elevator_MOTOR_ID, MotorType.kBrushless);
        elevatorMotor.restoreFactoryDefaults();
        elevatorMotor.setSmartCurrentLimit(2);
        forwardLimit = elevatorMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        reverseLimit = elevatorMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        forwardLimit.enableLimitSwitch(true);
        reverseLimit.enableLimitSwitch(true);
        tiltEncoder = elevatorMotor.getEncoder();
        pidController = elevatorMotor.getPIDController();
        pid.PIDCoefficientsElevator(pidController);
        pid.PIDToMax();
        logf("elevator System Setup kP for :%.6f\n", pid.kP);
    }

    public boolean setElevatorPos(double inches) {
        if (inches < 0 || inches > 40) {
            logf("****** Error attempted to set position out of range positon:%.1f\n", inches);
            return false;
        }
        double setPoint = inches * elevatorTicksPerInch;
        lastElevatorSetPoint = setPoint;
        lastElevatorPosition = inches;
        logf("Set position:%.2f set point:%f\n", inches, setPoint);
        pidController.setReference(setPoint, CANSparkMax.ControlType.kSmartMotion);
        double processVariable = tiltEncoder.getPosition();
        SmartDashboard.putNumber(" SP", setPoint);
        SmartDashboard.putNumber("Elev Pos", inches);
        SmartDashboard.putNumber("Elev Out", elevatorMotor.getAppliedOutput());
        SmartDashboard.putNumber("Process", processVariable);
        return true;
    }

    public double getLastElevatorPositionInches() {
        return lastElevatorPosition;
    }

    public double getLastElevatortSetPoint() {
        return lastElevatorSetPoint;
    }

    public void setBrakeMode(CANSparkMax motor, boolean mode) {
        motor.setIdleMode(mode ? IdleMode.kBrake : IdleMode.kCoast);
        logf("Brake mode: %s\n", motor.getIdleMode());
    }

    public double getElevatorPos() {
        return tiltEncoder.getPosition();
    }

    public double getElevatorLastPos() {
        return lastElevatorPosition;
    }

    public double getElevatorCurrent() {
        return elevatorMotor.getOutputCurrent();
    }

    public boolean getForwardLimitSwitch() {
        return forwardLimit.isPressed();
    }

    public boolean getReverseLimitSwitch() {
        return reverseLimit.isPressed();
    }

    // This method will be called once per scheduler run
    @Override
    public void periodic() {
        pidController.setReference(lastElevatorPosition, CANSparkMax.ControlType.kSmartMotion);
        if (Robot.count % 15 == 8) {
            double current = getElevatorCurrent();
            SmartDashboard.putNumber("ElevC", current);
            boolean forLimit = getForwardLimitSwitch();
            boolean revLimit = getReverseLimitSwitch();
            RobotContainer.leds.setLimitSwitchLed(Leds.ElevatorForward, forLimit);
            RobotContainer.leds.setLimitSwitchLed(Leds.ElevatorReverse, revLimit);
            SmartDashboard.putBoolean("ElevForL", forLimit);
            SmartDashboard.putBoolean("ElevRevL", revLimit);
            SmartDashboard.putNumber("ElevPos", getElevatorPos());
            SmartDashboard.putNumber("ElevLastPos", lastElevatorPosition);
            SmartDashboard.putNumber("Elev Out", elevatorMotor.getAppliedOutput());
        }
        if (RobotContainer.showPID == ShowPID.ELEVATOR && Robot.count % 15 == 12) {
            //  if ( Robot.count % 15 == 12) {
            pid.getPidCoefficientsFromDashBoard();
        }
    }
}
