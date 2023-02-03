package frc.robot.subsystems;

import static frc.robot.Robot.count;
import static frc.robot.utilities.Util.logf;
import static frc.robot.utilities.Util.round2;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

// setSensorPhase() should change the direction reported in the getSelectedSensor*() methods 
// (but not the SensorCollection methods).
// It should also change the direction reported for "PID0" in a self-test snapshot, 
// but not the position reported by "Quad/MagEnc(rel)" 

public class FXMotor {
    private TalonFX motor;
    private TalonFX followMotor;
    private String name;
    private int id;
    private int followId;
    private double lastSpeed = 0;
    private int lastPos = 0;
    private boolean myLogging = false;
    public ErrorCode errorCode;
    public ErrorCode errorCodeFollow;

    private boolean sensorPhase = false;
    private boolean motorInvert = false;
    // private FeedbackDevice feedBackDevice = FeedbackDevice.CTRE_MagEncoder_Relative;

    FXMotor(String name, int id, int followId, boolean logging) {
        this.name = name;
        this.id = id;
        this.followId = followId;
        myLogging = logging;
        motor = new TalonFX(this.id);
        errorCode = motor.configFactoryDefault();
        if (errorCode != ErrorCode.OK) {
            logf("????????? Motor %s Error: %s ??????????\n", name, errorCode);
        }
        if (followId > 0) {
            followMotor = new TalonFX(followId);
            errorCode = followMotor.configFactoryDefault();
            followMotor.follow(motor);
            if (errorCode != ErrorCode.OK) {
                logf("????????? Follow Motor %s Error: %s ??????????\n", name, errorCode);
                followId = -followId;
            }

        }

        motor.getSensorCollection().setIntegratedSensorPosition(0, 0);
        if (followId > 0)
            logf("Created %s motor ids:<%d,%d> firmware:<%d,%d> voltage:<%.1f,%.1f>\n", name, id, followId,
                    motor.getFirmwareVersion(), followMotor.getFirmwareVersion(), motor.getBusVoltage(),
                    followMotor.getBusVoltage());
        else
            logf("Created %s motor id:%d firmware:%d voltage:%.1f\n", name, id, motor.getFirmwareVersion(),
                    motor.getBusVoltage());

    }

    public String getName() {
        return name;
    }
    
    public int getPos() {
        return (int) motor.getSelectedSensorPosition();
    }

    void enableLimitSwitch(boolean forward, boolean reverse) {
        if (forward)
            motor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
        if (reverse)
            motor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    }

    boolean getForwardLimitSwitch() {
        return motor.getSensorCollection().isFwdLimitSwitchClosed() == 1;
    }

    boolean getReverseLimitSwitch() {
        return motor.getSensorCollection().isRevLimitSwitchClosed() == 1;
    }

    void setBrakeMode(boolean mode) {
        motor.setNeutralMode(mode ? NeutralMode.Brake : NeutralMode.Coast);
        if (followId > 0)
            followMotor.setNeutralMode(mode ? NeutralMode.Brake : NeutralMode.Coast);
    }

    void setPos(double position) {
        motor.set(ControlMode.Position, position);
    }

    public void setInverted(boolean invert) {
        this.motorInvert = invert;
        motor.setInverted(invert);
        if (followId > 0) {
            followMotor.setInverted(invert);
        }
    }

    public void setRampRate(double timeToFull) {
        motor.configOpenloopRamp(timeToFull);
        if (followId > 0) {
            followMotor.configOpenloopRamp(timeToFull);
        }
    }

    public double getLastSpeed() {
        return lastSpeed;
    }

    public double getActualSpeed() {
        return motor.getSelectedSensorVelocity(0);
    }

    public TalonFX getMotor() {
        return motor;
    }

    void periodic() {
        if (count % 50 == 0) {
            logPeriodic();
        }
        if (count % 500 == 0)
            updateSmart();
    }

    public void logPeriodic() {
        int pos = (int) motor.getSensorCollection().getIntegratedSensorPosition();
        if (pos != lastPos) {
            lastPos = pos;
            if (myLogging) {
                if (followId > 0) {
                    logf("%s motor sp:%.2f cur:%.2f temp:%.2f vel:%.2f pos:%.0f inv:%b senP:%b\n", name,
                            motor.getMotorOutputPercent(),
                            motor.getStatorCurrent(), motor.getTemperature(), motor.getSelectedSensorVelocity(), pos,
                            motorInvert, sensorPhase);
                    logf("%s follow sp:%.2f cur:%.2f temp:%.2f vel:%d pos:%d\n", name,
                            followMotor.getMotorOutputPercent(), followMotor.getStatorCurrent(),
                            followMotor.getTemperature(), followMotor.getSelectedSensorVelocity(),
                            (int) followMotor.getSensorCollection().getIntegratedSensorPosition());
                } else {
                    logf("%s motor sp:%.2f cur:%.2f temp:%.2f vel:%d pos:%d inv:%b senP:%b\n", name,
                            motor.getMotorOutputPercent(),
                            motor.getStatorCurrent(), motor.getTemperature(), motor.getSelectedSensorVelocity(),
                            (int) motor.getSensorCollection().getIntegratedSensorPosition(), motorInvert, sensorPhase);
                }
            }
        }

    }

    public void setCurrentLimit(int peakAmps, int continousAmps, int durationMilliseconds) {
        /*
         * Peak Current and Duration must be exceeded before current limit is activated.
         * When activated, current will be limited to Continuous Current. Set Peak
         * Current params to 0 if desired behavior is to immediately current-limit.
         */
        // talon.configPeakCurrentLimit(35, 10); /* 35 A */
        // talon.configPeakCurrentDuration(200, 10); /* 200ms */
        // talon.configContinuousCurrentLimit(30, 10); /* 30

        // SupplyCurrentLimitConfiguration(boolean enable, double currentLimit, double
        // triggerThresholdCurrent, double triggerThresholdTime)
        SupplyCurrentLimitConfiguration cl = new SupplyCurrentLimitConfiguration(true, peakAmps, continousAmps,
                durationMilliseconds);
        motor.configSupplyCurrentLimit(cl);
    }

    public void updateSmart() {
       // SmartDashboard.putNumber(name + " Pos", (int) motor.getSensorCollection().getIntegratedSensorPosition());
       // SmartDashboard.putNumber(name + " Cur", round2(motor.getStatorCurrent()));
    }

    public void setSpeed(double speed) {
        if (speed != lastSpeed) {
            motor.set(ControlMode.PercentOutput, speed);
            lastSpeed = speed;
        }
    }

    void forcePercentMode() {
        motor.set(ControlMode.PercentOutput, 0.001);
    }

    void zeroEncoder() {
        motor.setSelectedSensorPosition(0);
    }

    void setEncoderPosition(double position) {
        motor.getSensorCollection().setIntegratedSensorPosition((int) position, Constants.kTimeoutMs);
    }

    void setPositionPID(PID pid, FeedbackDevice feedBack) {
        //feedBackDevice = feedBack;
        setPositionPID(motor, 0, pid);
        PIDToFX(motor, pid, 0, Constants.kTimeoutMs);
    }

    void setVelocityPID(PID pid) {
        PIDToFX(motor, pid, 1, Constants.kTimeoutMs);
    }

    double getMotorVoltage() {
        return motor.getMotorOutputVoltage();
    }

    public void PIDToFX(TalonFX srx, PID pid, int slot, int timeout) {
        srx.config_kP(slot, pid.kP, timeout);
        srx.config_kI(slot, pid.kI, timeout);
        srx.config_kD(slot, pid.kD, timeout);
        srx.config_kF(slot, pid.kFF, timeout);
        srx.config_IntegralZone(slot, (int) pid.kIz, timeout);
        srx.configAllowableClosedloopError(slot, pid.allowableCloseLoopError, timeout);
        srx.configMaxIntegralAccumulator(slot, pid.maxIntegralAccumulation, timeout);
        logf("Setup %s PID for %s slot %d %s\n", pid.name, name, slot, pid.getPidData());
    }

    public double getError() {
        return motor.getClosedLoopError(0);
    }

    public void logMotorVCS() {
        if (Math.abs(lastSpeed) > .02) {
            logf("%s\n", getMotorsVCS(motor));
            if (followId > 0) {
                logf("%s\n", getMotorsVCS(followMotor));
            }
        }
    }

    public String getMotorsVCS(TalonFX motor) {
        if (Math.abs(lastSpeed) > .02) {
            double bussVoltage = motor.getBusVoltage();
            double outputVoltage = motor.getMotorOutputVoltage();
            double supplyCurrent = motor.getSupplyCurrent();
            double statorCurrent = motor.getStatorCurrent();
            return String.format("%s motor volts<%.2f:%.2f> cur<%.2f:%.2f> power<%.2f:%.2f> sp:%.3f", name, bussVoltage,
                    outputVoltage, supplyCurrent, statorCurrent, bussVoltage * supplyCurrent,
                    outputVoltage * statorCurrent, lastSpeed);
        }
        return name + "Not Running";
    }

    public double getMotorCurrent() {
        return motor.getStatorCurrent();
    }

    public void setSensorPhase(boolean phase) {
        sensorPhase = phase;
        motor.setSensorPhase(phase);
    }

    private void setPositionPID(TalonFX talon, int pidIdx, PID pid) {
        // Config the sensor used for Primary PID and sensor direction

        // talon.configSelectedFeedbackSensor(feedBackDevice, pidIdx,
        // Robot.config.kTimeoutMs);

        // Ensure sensor is positive when output is positive
        // talon.setSensorPhase(sensorPhase);

        // Set based on what direction you want forward/positive to be.
        // This does not affect sensor phase.

        // will need to do for follow motor -- take out and check climber code
        // talon.setInverted(motorInvert);

        /* Config the peak and nominal outputs, 12V means full */
        talon.configNominalOutputForward(0, Constants.kTimeoutMs);
        talon.configNominalOutputReverse(0, Constants.kTimeoutMs);
        talon.configPeakOutputForward(pid.kMaxOutput, Constants.kTimeoutMs);
        talon.configPeakOutputReverse(pid.kMinOutput, Constants.kTimeoutMs);

        // Config the allowable closed-loop error, Closed-Loop output will be neutral
        // within this range. See Table in Section 17.2.1 for native units per rotation.
        talon.configAllowableClosedloopError(0, pidIdx, Constants.kTimeoutMs);
    }
}