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

public class GrabberTiltSubsystem extends SubsystemBase {
    private static final int GRABBER_TILT_MOTOR_ID = 12;
    private double lastTiltPosition;
    private double lastTiltAngle = 0;
    private SparkMaxLimitSwitch tiltForwardLimit;
    private SparkMaxLimitSwitch tiltReverseLimit;
    private CANSparkMax grabberTiltMotor;
    private SparkMaxPIDController pidController;
    private RelativeEncoder tiltEncoder;
    private PID_MAX pid = new PID_MAX();

    public GrabberTiltSubsystem() {

        // Setup paramters for the tilt motor
        grabberTiltMotor = new CANSparkMax(GRABBER_TILT_MOTOR_ID, MotorType.kBrushless);
        grabberTiltMotor.restoreFactoryDefaults();
        grabberTiltMotor.setSmartCurrentLimit(2);
        tiltForwardLimit = grabberTiltMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        tiltReverseLimit = grabberTiltMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        tiltForwardLimit.enableLimitSwitch(true);
        tiltReverseLimit.enableLimitSwitch(true);
        tiltEncoder = grabberTiltMotor.getEncoder();
        pidController = grabberTiltMotor.getPIDController();
        pid.PIDCoefficientsTilt(pidController);
        pid.PIDToMax();
        pid.putPidCoefficientToDashBoard();
        logf("Grabber System Setup kP for Tilt:%.6f\n", pid.kP);

        // Limit currnet for Testing
        grabberTiltMotor.setSmartCurrentLimit(2);
    }

    public boolean setTiltAngle(double angle) {
        if (angle < 0 || angle > 50) {
            logf("****** Error attempted to set an angle to large or small angle:%.1f\n", angle);
            return false;
        }

        double setPoint = angle * (200000 / 360);
        setPoint = angle * 20;
        lastTiltAngle = angle;
        lastTiltPosition = setPoint;
        logf("Set angle:%.2f set point:%f kp:%f\n", angle, setPoint, pidController.getP());
        pidController.setReference(setPoint, CANSparkMax.ControlType.kSmartMotion);
        double processVariable = tiltEncoder.getPosition();
        SmartDashboard.putNumber("Tilt SP", setPoint);
        SmartDashboard.putNumber("Tilt Ang", angle);
        SmartDashboard.putNumber("Tilt Out", grabberTiltMotor.getAppliedOutput());
        SmartDashboard.putNumber("Process", processVariable);
        return true;
    }

    public double getLastTiltAngle() {
        return lastTiltAngle;
    }

    public void setBrakeMode(boolean mode) {
        grabberTiltMotor.setIdleMode(mode ? IdleMode.kBrake : IdleMode.kCoast);
        logf("Brake mode: %s\n", grabberTiltMotor.getIdleMode());
    }

    public double getTiltPos() {
        return tiltEncoder.getPosition();
    }

    public double getLastTiltPos() {
        return lastTiltPosition;
    }

    public double getTiltCurrent() {
        return grabberTiltMotor.getOutputCurrent();
    }

    public boolean getForwardLimitSwitchTilt() {
        return tiltForwardLimit.isPressed();
    }

    public boolean getReverseLimitSwitchTilt() {
        return tiltReverseLimit.isPressed();
    }

    // This method will be called once per scheduler run
    @Override
    public void periodic() {
        pidController.setReference(lastTiltPosition, CANSparkMax.ControlType.kSmartMotion); // TODO  see if this is needed
        if (RobotContainer.smartForElevator) {
            if (Robot.count % 15 == 5) {
                double current = getTiltCurrent();
                SmartDashboard.putNumber("TltCur", current);
                SmartDashboard.putBoolean("TltForL", getForwardLimitSwitchTilt());
                SmartDashboard.putBoolean("TltRevL", getReverseLimitSwitchTilt());
                SmartDashboard.putNumber("TltPos", getTiltPos());
                SmartDashboard.putNumber("TltLastPos", lastTiltPosition);
                SmartDashboard.putNumber("TltPwr", grabberTiltMotor.getAppliedOutput());
            }
            //if (RobotContainer.showPID == ShowPID.TILT && Robot.count % 30 == 10) {
            if ( Robot.count % 30 == 10) {
                pid.getPidCoefficientsFromDashBoard();
            }
        }
    }

}
