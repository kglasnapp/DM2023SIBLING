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
    private double lastElevatorInches = 0;
    private double lastElevatorSetPoint = 0;
    private LimitSwitch limitSwitch;
    private CANSparkMax elevatorMotor;
    private SparkMaxPIDController pidController;
    private RelativeEncoder distanceEncoder;
    private PID_MAX pid = new PID_MAX();
    private boolean homed = false;
    private double elevatorRotationsPerInch = 1; 
    private double current = 0;
    private GrabberTiltSubsystem grabberSubsystem;
    private double lastPower = 99;

    final private double MAX_CURRENT = 30;
    private int myCount = 0;

    enum STATE {
        IDLE, HOMING, READY, OVERCURRENT, OVERCURRENTSTOPPED
    }

    STATE state = STATE.IDLE;
    STATE lastState = null;

    public ElevatorSubsystem(GrabberTiltSubsystem grabberSubsystem) {
        // Setup paramters for the tilt motor
        elevatorMotor = new CANSparkMax(Elevator_MOTOR_ID, MotorType.kBrushless);
        elevatorMotor.restoreFactoryDefaults();
        elevatorMotor.setInverted(false); 
        setBrakeMode(elevatorMotor, true);
        elevatorMotor.setSmartCurrentLimit((int)MAX_CURRENT);
        limitSwitch = new LimitSwitch(elevatorMotor, "Elev", Leds.ElevatorForward, Leds.ElevatorReverse);
        distanceEncoder = elevatorMotor.getEncoder();
        distanceEncoder.setPosition(0);
        pidController = elevatorMotor.getPIDController();
        pid.PIDCoefficientsElevator(pidController);
        pid.PIDToMax();
        logf("Elevator System Setup kP for :%.6f Conversion:%.2f Counts per Rev:%d\n", pid.kP,
                distanceEncoder.getPositionConversionFactor(), distanceEncoder.getCountsPerRevolution());
        this.grabberSubsystem = grabberSubsystem;

    }

    public boolean setElevatorPos(double inches) {
        if (inches < 0 || inches > 130) {
            logf("****** Error attempted to set position out of range positon:%.1f\n", inches);
            return false;
        }
        if (grabberSubsystem.isElevatorSafeToMove(inches)) {
            
            double setPoint = inches * elevatorRotationsPerInch;
            lastElevatorSetPoint = setPoint;
            lastElevatorInches = inches;
            logf("Set Elevator position:%.2f set point:%f\n", inches, setPoint);
            pidController.setReference(setPoint, CANSparkMax.ControlType.kSmartMotion);
            SmartDashboard.putNumber("Elev SP", setPoint);
            SmartDashboard.putNumber("Elev Inch", inches);
            return true;
        } else {
            logf("***** Elevator not safe to move angle:%.2f\n", grabberSubsystem.getAbsEncoder());
            return false;
        }
    }

    public void setPower(double value, boolean homing) {
        if (grabberSubsystem.isElevatorSafeToMove(value) || homing) {
            if (lastPower != value || value == 0) {
                logf("Elevator set a new power %.2f\n", value);
                elevatorMotor.set(value);
                lastPower = value;
            }
        }
    }

    public double getLastElevatorPositionInches() {
        return lastElevatorInches;
    }

    public double getLastElevatortSetPoint() {
        return lastElevatorSetPoint;
    }

    public void setBrakeMode(CANSparkMax motor, boolean mode) {
        motor.setIdleMode(mode ? IdleMode.kBrake : IdleMode.kCoast);
        logf("Brake mode: %s\n", motor.getIdleMode());
    }

    public double getElevatorPosRevs() {
        return distanceEncoder.getPosition();
    }

    public double getElevatorLastPosInches() {
        return lastElevatorInches;
    }

    public double getElevatorCurrent() {
        return elevatorMotor.getOutputCurrent();
    }

    public boolean atSetPoint() {
        double error = distanceEncoder.getPosition() - lastElevatorSetPoint;
        if (Robot.count % 10 == 5) {
            SmartDashboard.putNumber("EleErr", error);
        }
        // Note error is in revolutions
        return Math.abs(error) < .5;
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

    public void setEncoderRevs(double value) {
        distanceEncoder.setPosition(value);
    }

    double lastSetPointForLogging = 0;
    // This method will be called once per scheduler run
    @Override
    public void periodic() {
        limitSwitch.periodic();
        if (atSetPoint() && lastSetPointForLogging != lastElevatorSetPoint) {
            logf("Elevator at set point:%.2f requested set point:%.2f\n", distanceEncoder.getPosition(),
                    lastElevatorSetPoint);
            lastSetPointForLogging = lastElevatorSetPoint;
        }
        current = getElevatorCurrent();
        doHomingAndMonitor(current);
        if (state == STATE.READY) {
            pidController.setReference(lastElevatorSetPoint, CANSparkMax.ControlType.kSmartMotion);
        }
        if (Robot.count % 15 == 8) {
            SmartDashboard.putNumber("ElevCur", round2(current));
            SmartDashboard.putNumber("ElevPos", round2(getElevatorPosRevs()));
            SmartDashboard.putNumber("ElevLastPos", lastElevatorInches);
            SmartDashboard.putNumber("ElevPwr", round2(elevatorMotor.getAppliedOutput()));
            SmartDashboard.putNumber("ElevVel", round2(distanceEncoder.getVelocity()));
        }
        if (RobotContainer.showPID == ShowPID.ELEVATOR && Robot.count % 15 == 12) {
            if (Robot.count % 15 == 12) {
                // TODO pid.getPidCoefficientsFromDashBoard();
            }
        }
    }

    private void doHomingAndMonitor(double current) {
        if (state != lastState) {
            logf("Elevator State Changed state:%s current:%.3f myCount:%d angle:%.2f\n", state, current, myCount,
                    grabberSubsystem.getAbsEncoder());
            SmartDashboard.putString("Elev State", state.toString());
            lastState = state;
        }
        switch (state) {
            case IDLE:
                // Can home only if intake is retracted
                if (!grabberSubsystem.isRetracted()) {
                    if (Robot.count % 250 == 12) {
                        logf("Can't Home since the grabber is not retracted angle:%.3f\n",
                                grabberSubsystem.getAbsEncoder());
                    }
                    break;
                }
                state = STATE.HOMING;
                break;
            case HOMING:
                if (getReverseLimitSwitch()) {
                    // At home so stop motor and indicate homed
                    setPower(0, true);
                    setHomed(true);
                    setEncoderRevs(0);
                    state = STATE.READY;
                    logf("Elevator is homed\n");
                    break;
                }
                if (current > MAX_CURRENT) {
                    logf("Elevator overcurrent detected while homing current:%.2f\n", current);
                    myCount = 5; // Wait 100 ms to see if over current remains
                    state = STATE.OVERCURRENT;
                    break;
                }
                setPower(-.2, true);
                break;
            case READY:
                if (current > MAX_CURRENT) {
                    logf("Elevator overcurrent detected while ready current:%.2f\n", current);
                    myCount = 5; // Wait 100 ms to see if over current remains
                    state = STATE.OVERCURRENT;                    
                }
                break;
            case OVERCURRENT:
                myCount--;
                if (myCount < 0) {
                    if (current > MAX_CURRENT) {
                        setPower(0, true);
                        myCount = 20; // Wait 400 ms to restart
                        state = STATE.OVERCURRENTSTOPPED;
                    } else {
                        state = STATE.HOMING;
                    }
                }
                break;
            case OVERCURRENTSTOPPED:
                myCount--;
                if (myCount < 0) {
                    if (current > MAX_CURRENT) {
                        // If curent remains high continue to wait
                        logf("***** Elevator current remains high -- current:%.2f\n", current);
                        myCount = 20; // Wait another 400 ms for current to go low
                        break;
                    }
                    // Current seems to have stablize restore last task
                    state = STATE.HOMING;
                }
                break;
        }
    }

}
