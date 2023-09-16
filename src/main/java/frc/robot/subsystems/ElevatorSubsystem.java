package frc.robot.subsystems;

import static frc.robot.Util.logf;
import static frc.robot.Util.round2;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.ShowPID;
import frc.robot.subsystems.LedSubsystem.Leds;
import frc.robot.utilities.LimitSwitch;

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
    private LimitSwitch limitSwitch;
    private CANSparkMax elevatorMotor;
    private SparkMaxPIDController pidController;
    private RelativeEncoder distanceEncoder;
    private PID_MAX pid = new PID_MAX();
    private boolean homed = false;
    private double elevatorRotationsPerInch = 1;  // TODO Fix when connect to real robot
    private double current = 0;

    public ElevatorSubsystem() {

        // Setup paramters for the tilt motor
        elevatorMotor = new CANSparkMax(Elevator_MOTOR_ID, MotorType.kBrushless);
        elevatorMotor.restoreFactoryDefaults();
        elevatorMotor.setSmartCurrentLimit(2);
        limitSwitch = new LimitSwitch(elevatorMotor, "Elev", Leds.ElevatorForward, Leds.ElevatorReverse);
        distanceEncoder = elevatorMotor.getEncoder();
        distanceEncoder.setPosition(0);
        pidController = elevatorMotor.getPIDController();
        pid.PIDCoefficientsElevator(pidController);
        pid.PIDToMax();
        logf("Elevator System Setup kP for :%.6f Conversion:%.2f Counts per Rev:%d\n", pid.kP,
                distanceEncoder.getPositionConversionFactor(), distanceEncoder.getCountsPerRevolution());
    }

    public boolean setElevatorPos(double inches) {
        if (inches < 0 || inches > 40) {
            logf("****** Error attempted to set position out of range positon:%.1f\n", inches);
            return false;
        }
        double setPoint = inches * elevatorRotationsPerInch;
        lastElevatorSetPoint = setPoint;
        lastElevatorPosition = inches;
        logf("Set Elevator position:%.2f set point:%f\n", inches, setPoint);
        pidController.setReference(setPoint, CANSparkMax.ControlType.kSmartMotion);
        SmartDashboard.putNumber("Elev SP", setPoint);
        SmartDashboard.putNumber("Elev Inch", inches);
        return true;
    }

    public void setPower(double value) {
        elevatorMotor.set(value);
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
        return distanceEncoder.getPosition();
    }

    public double getElevatorLastPos() {
        return lastElevatorPosition;
    }

    public double getElevatorCurrent() {
        return current;
    }

    public boolean atSetPoint() {
        double error = distanceEncoder.getPosition() - lastElevatorPosition;
        if (Robot.count % 10 == 5) {
            SmartDashboard.putNumber("EleErr", error);
        }
        // Note error is in revolutions
        return Math.abs(error) < .1;
    }

    public boolean getForwardLimitSwitch() {
        return limitSwitch.getForward();
    }

    public boolean getReverseLimitSwitch() {
        return limitSwitch.getReverse();
    }

    public boolean getHomed() {
        return homed;
    }

    public void setHomed(boolean value) {
        homed = value;
    }

    public void setEncoder(double value) {
        distanceEncoder.setPosition(value);
    }

    // This method will be called once per scheduler run
    @Override
    public void periodic() {
        limitSwitch.periodic();
        current = getElevatorCurrent();
        //pidController.setReference(lastElevatorPosition, CANSparkMax.ControlType.kSmartMotion);
        if (Robot.count % 15 == 8) {

            SmartDashboard.putNumber("ElevCur", round2(current));
            SmartDashboard.putNumber("ElevPos", round2(getElevatorPos()));
            SmartDashboard.putNumber("ElevLastPos", lastElevatorPosition);
            SmartDashboard.putNumber("ElevPwr", round2(elevatorMotor.getAppliedOutput()));
        }
        if (RobotContainer.showPID == ShowPID.ELEVATOR && Robot.count % 15 == 12) {
            if (Robot.count % 15 == 12) {
                pid.getPidCoefficientsFromDashBoard();
            }
        }
    }
}
