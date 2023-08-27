package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

import static frc.robot.Util.logf;

public class RearGrabberSubsystem extends SubsystemBase {
    private static final int REAR_GRABBER_TILT_MOTOR_ID = 21;
    private static final int REAR_GRABBER_INTAKE_MOTOR_ID = 20;

    // private static double startGrab = 0;
    private PID grabberPid;
    private TalonSRX rearGrabberTiltMotor;
    private TalonSRX rearGrabberIntakeMotor;
    private double lastGrabberStopPosition;
    private double lastIntakeSpeed = 0;

    public RearGrabberSubsystem() {
        // Setups for Tilt Motor
        rearGrabberTiltMotor = new TalonSRX(REAR_GRABBER_TILT_MOTOR_ID);
        rearGrabberTiltMotor.configFactoryDefault();
        rearGrabberTiltMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 5, 5, .2));
        rearGrabberTiltMotor.configContinuousCurrentLimit(10);
        enableLimitSwitch(rearGrabberTiltMotor);
        setBrakeMode(rearGrabberTiltMotor, true);
        grabberPid = new PID("RGrbPos", .6, .001, 3, 0.0, 0, -0.3, 0.3, false);
        grabberPid = new PID("RGrbPos", 1.2, .0005, 5, 0.0, 0, -0.6, 0.6, false);
        PIDToSRX(rearGrabberTiltMotor, grabberPid, 0, 20);
        rearGrabberTiltMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog);

        // Setup for Intake motor
        rearGrabberIntakeMotor = new TalonSRX(REAR_GRABBER_INTAKE_MOTOR_ID);
        rearGrabberIntakeMotor.configFactoryDefault();
        rearGrabberIntakeMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 5, 5, .2));
        rearGrabberIntakeMotor.configContinuousCurrentLimit(10);
        setBrakeMode(rearGrabberIntakeMotor, true);
        logf("Start Front Grabber\n");
    }

    void setBrakeMode(TalonSRX motor, boolean mode) {
        motor.setNeutralMode(mode ? NeutralMode.Brake : NeutralMode.Coast);
    }

    void enableLimitSwitch(TalonSRX motor) {
        motor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
                LimitSwitchNormal.NormallyOpen);
        motor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
                LimitSwitchNormal.NormallyOpen);
    }

    public double getTiltPos() {
        return rearGrabberTiltMotor.getSensorCollection().getAnalogInRaw();
    }

    public double getLastTiltPos() {
        return lastGrabberStopPosition;
    }

    public void setTiltPosition(double position) {
        rearGrabberTiltMotor.set(ControlMode.Position, position);
        lastGrabberStopPosition = position;
    }

    public void setTiltPower(double speed) {
        // logf("------ setting speed for grabber: %.2f\n",speed);
        if (speed == 0) {
            rearGrabberTiltMotor.set(ControlMode.Disabled, 0);
        } else {
            rearGrabberTiltMotor.set(ControlMode.PercentOutput, speed);
        }
    }

    public double getTiltCurrent() {
        return rearGrabberTiltMotor.getSupplyCurrent();
    }

    private boolean getForwardLimitSwitchTilt() {
        return rearGrabberTiltMotor.getSensorCollection().isFwdLimitSwitchClosed();
    }

    private boolean getReverseLimitSwitchTilt() {
        return rearGrabberTiltMotor.getSensorCollection().isRevLimitSwitchClosed();
    }

    public void setIntakePower(double speed) {
        if (lastIntakeSpeed != speed) {
            rearGrabberIntakeMotor.set(ControlMode.PercentOutput, speed);
            logf("Grabber Intake SP:%.2f\n", speed);
            lastIntakeSpeed = speed;
        }
    }

    private void PIDToSRX(TalonSRX srx, PID pid, int slot, int timeout) {
        srx.config_kP(slot, pid.kP, timeout);
        srx.config_kI(slot, pid.kI, timeout);
        srx.config_kD(slot, pid.kD, timeout);
        srx.config_kF(slot, pid.kFF, timeout);
        srx.config_IntegralZone(slot, (int) pid.kIz, timeout);
        srx.configAllowableClosedloopError(slot, pid.allowableCloseLoopError, timeout);
        srx.configMaxIntegralAccumulator(slot, pid.maxIntegralAccumulation, timeout);
        logf("Setup %s PID slot %d %s\n", pid.name, slot, pid.getPidData());
    }

    // This method will be called once per scheduler run
    @Override
    public void periodic() {
        if (Robot.count % 15 == 5) {
            double statCurrent = rearGrabberTiltMotor.getStatorCurrent();
            SmartDashboard.putNumber("GrStC", statCurrent);
            double current = rearGrabberTiltMotor.getSupplyCurrent();
            SmartDashboard.putNumber("GrSpC", current);
            boolean forwardLimit = getForwardLimitSwitchTilt();
            SmartDashboard.putBoolean("GrForL", forwardLimit);
            boolean reverseLimit = getReverseLimitSwitchTilt();
            SmartDashboard.putBoolean("GrRevL", reverseLimit);
            double position = getTiltPos();
            SmartDashboard.putNumber("GrPos", position);
            SmartDashboard.putNumber("GrLastPos", lastGrabberStopPosition);
        }
    }
}