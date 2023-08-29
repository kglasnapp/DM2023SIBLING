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
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;
    public int count = 0;
    public int myMode = 0;

    public GrabberSubsystem() {

        // Setup paramters for the tilt motor
        grabberTiltMotor = new CANSparkMax(GRABBER_TILT_MOTOR_ID, MotorType.kBrushless);
        grabberTiltMotor.restoreFactoryDefaults();
        tiltForwardLimit = grabberTiltMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);
        tiltReverseLimit = grabberTiltMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);
        tiltForwardLimit.enableLimitSwitch(false);
        tiltReverseLimit.enableLimitSwitch(false);
        tiltEncoder = grabberTiltMotor.getEncoder();
        pidController = grabberTiltMotor.getPIDController();
        setDefaultPIPCoefficients();
        PIDToMax();

        // Setup parametere for the grabber motor
        grabberIntakeMotor = new CANSparkMax(GRABBER_INTAKE_MOTOR_ID, MotorType.kBrushless);
        grabberIntakeMotor.restoreFactoryDefaults();
    }

    public boolean setTiltAngle(double angle) {
        if (angle < 0 || angle > 50) {
            logf("****** Error attempted to set an angle to large or small angle:%.1f\n", angle);
            return false;
        }
        double setPoint = angle / 1000;
        lastTiltAngle = angle;
        lastTiltPosition = setPoint;
        pidController.setReference(setPoint, CANSparkMax.ControlType.kSmartMotion);
        SmartDashboard.putNumber("Tilt SP", setPoint);
        SmartDashboard.putNumber("Tilt Out", grabberTiltMotor.getAppliedOutput());
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

    private void setDefaultPIPCoefficients() {
        kP = 5e-5;
        kI = 1e-6;
        kD = 0;
        kIz = 0;
        kFF = 0.000156;
        kMaxOutput = 1;
        kMinOutput = -1;
        // Smart Motion Coefficients
        maxRPM = 5700; // maxPRM  velocity mode only
        maxVel = 2000; // maxVel  for velocity mode only
        maxAcc = 1500;
    }

    private void PIDToMax() {
        // set PID coefficients
        pidController.setP(kP);
        pidController.setI(kI);
        pidController.setD(kD);
        pidController.setIZone(kIz);
        pidController.setFF(kFF);
        pidController.setOutputRange(kMinOutput, kMaxOutput);

    }

    private void getPidCoefficientsFromDashBoard() {
        // read PID coefficients from SmartDashboard
        double p = SmartDashboard.getNumber("P Gain", 0);
        double i = SmartDashboard.getNumber("I Gain", 0);
        double d = SmartDashboard.getNumber("D Gain", 0);
        double iz = SmartDashboard.getNumber("I Zone", 0);
        double ff = SmartDashboard.getNumber("Feed Forward", 0);
        double max = SmartDashboard.getNumber("Max Output", 0);
        double min = SmartDashboard.getNumber("Min Output", 0);
        double maxV = SmartDashboard.getNumber("Max Velocity", 0);
        double minV = SmartDashboard.getNumber("Min Velocity", 0);
        double maxA = SmartDashboard.getNumber("Max Acceleration", 0);
        double allE = SmartDashboard.getNumber("Allowed Closed Loop Error", 0);
        // if PID coefficients on SmartDashboard have changed, write new values to controller
        if ((p != kP)) {
            pidController.setP(p);
            kP = p;
        }
        if ((i != kI)) {
            pidController.setI(i);
            kI = i;
        }
        if ((d != kD)) {
            pidController.setD(d);
            kD = d;
        }
        if ((iz != kIz)) {
            pidController.setIZone(iz);
            kIz = iz;
        }
        if ((ff != kFF)) {
            pidController.setFF(ff);
            kFF = ff;
        }
        if ((max != kMaxOutput) || (min != kMinOutput)) {
            pidController.setOutputRange(min, max);
            kMinOutput = min;
            kMaxOutput = max;
        }
        if ((maxV != maxVel)) {
            pidController.setSmartMotionMaxVelocity(maxV, 0);
            maxVel = maxV;
        }
        if ((minV != minVel)) {
            pidController.setSmartMotionMinOutputVelocity(minV, 0);
            minVel = minV;
        }
        if ((maxA != maxAcc)) {
            pidController.setSmartMotionMaxAccel(maxA, 0);
            maxAcc = maxA;
        }
        if ((allE != allowedErr)) {
            pidController.setSmartMotionAllowedClosedLoopError(allE, 0);
            allowedErr = allE;
        }
    }

    // This method will be called once per scheduler run
    @Override
    public void periodic() {
        if (Robot.count % 15 == 5) {
            double current = getTiltCurrent();
            SmartDashboard.putNumber("GrStC", current);
            SmartDashboard.putBoolean("GrForL", getForwardLimitSwitchTilt());
            SmartDashboard.putBoolean("GrRevL", getReverseLimitSwitchTilt());
            SmartDashboard.putNumber("GrPos", getTiltPos());
            SmartDashboard.putNumber("GrLastPos", lastTiltPosition);
        }
        if (Robot.count % 15 == 10) {
            getPidCoefficientsFromDashBoard();
        }
    }
}