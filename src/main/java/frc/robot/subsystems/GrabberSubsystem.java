package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import static frc.robot.utilities.Util.logf;

public class GrabberSubsystem extends SubsystemBase {
    public static final int GRABBER_MOTOR_ID = 15;
   
    private PID grabberPid;
    public TalonSRX grabberMotor;
    public double lastGrabberStopPosition;
    private double lastPowerLevel  = 0;

    public GrabberSubsystem() {
        grabberMotor = new TalonSRX(GRABBER_MOTOR_ID);
        grabberMotor.configFactoryDefault();
        setCurrentLimits();
        enableLimitSwitch();
        setBrakeMode(grabberMotor, true);
        grabberPid = new PID("GrbPos", .15, 0, 0, 0, 0, -0.3, 0.3, false);
        PIDToSRX(grabberMotor, grabberPid, 0, Constants.kTimeoutMs);
        logf("Grabber Motor Enabled\n");       
    }

    void setBrakeMode(TalonSRX motor, boolean mode) {
        motor.setNeutralMode(mode ? NeutralMode.Brake : NeutralMode.Coast);
    }

    void enableLimitSwitch() {
       grabberMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
                LimitSwitchNormal.NormallyOpen);
        grabberMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
                LimitSwitchNormal.NormallyOpen);
    }

    public double getGrabberPos() {
        return grabberMotor.getSensorCollection().getQuadraturePosition();
    }

    public void setMotorToPosition( double position) {
        grabberMotor.set(ControlMode.Position, position);
            lastGrabberStopPosition = position;
    }

    public void setCurrentLimits() {
        SupplyCurrentLimitConfiguration currLimitCfg = new SupplyCurrentLimitConfiguration(true, 5, 5, .2);
        grabberMotor.configSupplyCurrentLimit(currLimitCfg);
        grabberMotor.configContinuousCurrentLimit(10);
    }

    public double getSupplyCurrent() {
        return grabberMotor.getSupplyCurrent();
    }

    public double getStatorCurrent() {
        return grabberMotor.getStatorCurrent();
    }

    public double getLastPowerLevel() {
        return lastPowerLevel;
    }
    public void setGrabberPower(double speed) {
        if (speed == 0) {
            grabberMotor.set(ControlMode.Disabled, 0);
        } else {
            grabberMotor.set(ControlMode.PercentOutput, speed);
        }
        lastPowerLevel = speed;
    }

    public void zeroEncoder() {
        grabberMotor.getSensorCollection().setQuadraturePosition(0, Constants.kTimeoutMs);
    }

    public void setEncoderPosition(int position) {
        grabberMotor.getSensorCollection().setQuadraturePosition(position, Constants.kTimeoutMs);
    }

    public boolean getForwardLimitSwitch() {
        return grabberMotor.getSensorCollection().isFwdLimitSwitchClosed();
    }

    public boolean getReverseLimitSwitch() {
        return grabberMotor.getSensorCollection().isRevLimitSwitchClosed();
    }

    public void PIDToSRX(TalonSRX srx, PID pid, int slot, int timeout) {
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
        // Display Grabber Data
        if (Robot.count % 15 == 5) {
            double statCurrent = grabberMotor.getStatorCurrent();
            SmartDashboard.putNumber("GrbStatCur", statCurrent);
            //double current = grabberMotor.getSupplyCurrent();
            //SmartDashboard.putNumber("GrbSuppCur", current);
            //boolean forwardLimit = getForwardLimitSwitch();
            //SmartDashboard.putBoolean("GrbForL", forwardLimit);
            //boolean reverseLimit = getReverseLimitSwitch();
            //SmartDashboard.putBoolean("GrbRevL", reverseLimit);
            //double position = getGrabberPos();
            //SmartDashboard.putNumber("GrbPos", position);
        }

    }
}