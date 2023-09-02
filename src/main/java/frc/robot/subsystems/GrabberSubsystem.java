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

public class GrabberSubsystem extends SubsystemBase {
    private static final int GRABBER_TILT_MOTOR_ID = 11;
    private static final int GRABBER_INTAKE_MOTOR_ID = 10;
    private double lastTiltPosition;
    private double lastTiltAngle = 0;
    private double lastIntakeSpeed = 0;
    private SparkMaxLimitSwitch tiltForwardLimit;
    private SparkMaxLimitSwitch tiltReverseLimit;
    private CANSparkMax grabberTiltMotor;
    private CANSparkMax grabberIntakeMotor;
    private SparkMaxPIDController pidController;
    private RelativeEncoder tiltEncoder;
    private PID_MAX pid = new PID_MAX();

    public GrabberSubsystem() {

        // Setup paramters for the tilt motor
        grabberTiltMotor = new CANSparkMax(GRABBER_TILT_MOTOR_ID, MotorType.kBrushless);
        grabberTiltMotor.restoreFactoryDefaults();
        tiltForwardLimit = grabberTiltMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        tiltReverseLimit = grabberTiltMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        tiltForwardLimit.enableLimitSwitch(true);
        tiltReverseLimit.enableLimitSwitch(true);
        tiltEncoder = grabberTiltMotor.getEncoder();
        pidController = grabberTiltMotor.getPIDController();
        pid.PIDCoefficientsTilt(pidController);
        pid.PIDToMax();

        // Setup parametere for the grabber motor
        grabberIntakeMotor = new CANSparkMax(GRABBER_INTAKE_MOTOR_ID, MotorType.kBrushless);
        grabberIntakeMotor.restoreFactoryDefaults();
        pid.putPidCoefficientToDashBoard();
        logf("Grabber System Setup kP for Tilt:%.6f\n", pid.kP);
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
        logf("Set angle:%.2f set point:%f\n", angle, setPoint);
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

    public void setBrakeMode(CANSparkMax motor, boolean mode) {
        motor.setIdleMode(mode ? IdleMode.kBrake : IdleMode.kCoast);
        logf("Brake mode: %s\n", motor.getIdleMode());
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

    private boolean getForwardLimitSwitchTilt() {
        return tiltForwardLimit.isPressed();
    }

    private boolean getReverseLimitSwitchTilt() {
        return tiltReverseLimit.isPressed();
    }

    public void setIntakePower(double speed) {
        if (lastIntakeSpeed != speed) {
            grabberIntakeMotor.set(speed);
            logf("Grabber Intake speed:%.2f\n", speed);
            lastIntakeSpeed = speed;
        }
    }

    // This method will be called once per scheduler run
    @Override
    public void periodic() {
        pidController.setReference(lastTiltPosition, CANSparkMax.ControlType.kSmartMotion);
        if (RobotContainer.smartForElevator) {
          if (Robot.count % 15 == 5) {
                double current = getTiltCurrent();
                SmartDashboard.putNumber("GrStC", current);
                SmartDashboard.putBoolean("GrForL", getForwardLimitSwitchTilt());
                SmartDashboard.putBoolean("GrRevL", getReverseLimitSwitchTilt());
                SmartDashboard.putNumber("GrPos", getTiltPos());
                SmartDashboard.putNumber("GrLastPos", lastTiltPosition);
                SmartDashboard.putNumber("Tilt Out", grabberTiltMotor.getAppliedOutput());
            }
            if (Robot.count % 15 == 10) {
                pid.getPidCoefficientsFromDashBoard();
            }
        }
    }

}