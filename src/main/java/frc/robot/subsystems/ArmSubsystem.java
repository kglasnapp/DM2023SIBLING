package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

import static frc.robot.utilities.Util.logf;

public class ArmSubsystem extends SubsystemBase {
    public static final double SHOULDER_TO_ELBOW = 10;
    public static final double ELBOW_TO_WRIST = 10;
    public static final int DEVICE_NUMBER = 1;
    public static final double TICKS_PER_ROTATION = 2048.0 * 4; // 4 is the gear
    public static final int SHOULDER_MOTOR_ID = 10;
    public static final int EXTENDER_MOTOR_ID = 11;

    public static final int SHOULDER_MOTOR_MASTER = 14;
    public static final int SHOULDER_MOTOR_SLAVE = 13;

    private PID shoulderPid;
    private PID extenderPid;
    // private double lastSpeed = 0;
    // private double lastJoy = 0;
    
    public TalonFX shoulderMotor;
    public TalonFX extenderMotor;

    public double lastExtenderStopPosition;
    public double lastShoulderStopPosition;

    boolean useTwoMotors = false;

    public ArmSubsystem() {
        shoulderPid = new PID("ShlPos", .1, 0, 0, 0, 0, -0.7, 0.7, false);
        if (useTwoMotors) {
            shoulderMotor = new TalonFX(SHOULDER_MOTOR_MASTER);
            TalonFX motor2 = new TalonFX(SHOULDER_MOTOR_SLAVE);
            shoulderMotor.setInverted(true);
            
        //    motor2.follow(shoulderMotor);
            motor2.configFactoryDefault();
            motor2.setInverted(true);
            setBrakeMode(motor2, true);
            shoulderMotor.configNeutralDeadband(0.04);
            //PIDToFX(motor2, shoulderPid, 0, Constants.kTimeoutMs);
            motor2.set(ControlMode.Follower, SHOULDER_MOTOR_MASTER);
        } else {
            shoulderMotor = new TalonFX(10);
            shoulderMotor.setInverted(true);
        }
        shoulderMotor.configFactoryDefault();
        setCurrentLimits(shoulderMotor,10);
        enableLimitSwitch(shoulderMotor);
        setBrakeMode(shoulderMotor, true);
        
        PIDToFX(shoulderMotor, shoulderPid, 0, Constants.kTimeoutMs);
        
        logf("Shoulder Motor Enabled\n");

        extenderMotor = new TalonFX(EXTENDER_MOTOR_ID);
        extenderMotor.configFactoryDefault();
        setCurrentLimits(extenderMotor,10);
        enableLimitSwitch(extenderMotor);
        setBrakeMode(extenderMotor, true);
        //extenderPid = new PID("ExtPos", 0.5, 0, 0, 0, 0, -0.6, 0.6, false);
        extenderPid = new PID("ExtPos", 0.5, 0, 0, 0, 0, -.3, .3, false);
        PIDToFX(extenderMotor, extenderPid, 0, Constants.kTimeoutMs);
        logf("Extender Motor Enabled\n");
    }

    void setBrakeMode(TalonFX motor, boolean mode) {
        motor.setNeutralMode(mode ? NeutralMode.Brake : NeutralMode.Coast);
    }

    void enableLimitSwitch(TalonFX motor) {
        motor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
                LimitSwitchNormal.NormallyOpen);
        motor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
                LimitSwitchNormal.NormallyOpen);
    }

    public double getShoulderPos() {
        return shoulderMotor.getSelectedSensorPosition(); //getSensorCollection().getIntegratedSensorPosition();
    }

    public double getExtenderPos() {
        return extenderMotor.getSelectedSensorPosition(); //getSensorCollection().getIntegratedSensorPosition();
    }

    public double getShoulderRevs() {
        return getShoulderPos() / (2000 * 48);
    }

    public double getExtenderRevs() {
        return getExtenderPos() / (2000 * 4);
    }

    public double getExtenderInches() {
        double inchesPerRototion = 2 * Math.PI;
        double position = TICKS_PER_ROTATION * inchesPerRototion;
        return extenderMotor.getSensorCollection().getIntegratedSensorPosition() / position;
    }

    public void setMotorToPosition(TalonFX motor, double position) {
        motor.set(ControlMode.Position, position);
        if (motor == extenderMotor) {
            lastExtenderStopPosition = position;
        } else if (motor == shoulderMotor) {
            lastShoulderStopPosition = position;
        }
    }

    public void setCurrentLimits(TalonFX motor, double current) {
        SupplyCurrentLimitConfiguration currLimitCfg = new SupplyCurrentLimitConfiguration(true, current, current, .2);
        motor.configSupplyCurrentLimit(currLimitCfg);
       StatorCurrentLimitConfiguration statCurrentLimitCfg = new StatorCurrentLimitConfiguration(true, current, current, .2);
       motor.configGetStatorCurrentLimit(statCurrentLimitCfg);
    }

    public void setShoulderSpeed(double speed) {
        if (speed == 0) {
            //shoulderMotor.set(ControlMode.Disabled, 0);
            System.out.println("Setting velocity to zero (((()))))))");
            //shoulderMotor.selectProfileSlot(EXTENDER_MOTOR_ID, DEVICE_NUMBER);
            shoulderMotor.set(ControlMode.Velocity, 0);        
            //lastShoulderStopPosition = shoulderMotor.getSelectedSensorPosition();
        } else {
            shoulderMotor.set(ControlMode.PercentOutput, speed);
        }
    }

    public void setExtenderSpeed(double speed) {
        if (speed == 0) {
            extenderMotor.set(ControlMode.Disabled, 0);
            //lastExtenderStopPosition = extenderMotor.getSelectedSensorPosition();
        } else {
            extenderMotor.set(ControlMode.PercentOutput, speed);
        }
    }

    public void setShoulderVelocity(double velocity) {
        if (velocity == 0) {
            shoulderMotor.set(ControlMode.Disabled, 0);
           // lastShoulderStopPosition = shoulderMotor.getSelectedSensorPosition();
        } else {
            shoulderMotor.set(ControlMode.Velocity, velocity);
        }
    }

    public void setExtenderVelocity(double velocity) {
        if (velocity == 0) {
            extenderMotor.set(ControlMode.Disabled, 0);
           // lastExtenderStopPosition = extenderMotor.getSelectedSensorPosition();
        } else {
            extenderMotor.set(ControlMode.Velocity, velocity);
        }
    }

    public void zeroEncoder(TalonFX motor) {
        motor.setSelectedSensorPosition(0.0);
        //getSensorCollection().setIntegratedSensorPosition(0.0, Constants.kTimeoutMs);
    }

    public void setEncoderPosition(TalonFX motor, double position) {
        motor.setSelectedSensorPosition(position);
        //motor.set(ControlMode.Velocity, 0);
        //getSensorCollection().setIntegratedSensorPosition(position, Constants.kTimeoutMs);
        if (motor == shoulderMotor) {
            //lastShoulderStopPosition = shoulderMotor.getSelectedSensorPosition();
        } else if (motor == extenderMotor) {
            //lastExtenderStopPosition = extenderMotor.getSelectedSensorPosition();
        }
    }

    public boolean getForwardLimitSwitch(TalonFX motor) {
        return motor.getSensorCollection().isFwdLimitSwitchClosed() == 1;
    }

    public boolean getReverseLimitSwitch(TalonFX motor) {
        return motor.getSensorCollection().isRevLimitSwitchClosed() == 1;
    }

    public void PIDToFX(TalonFX srx, PID pid, int slot, int timeout) {
        srx.config_kP(slot, pid.kP, timeout);
        srx.config_kI(slot, pid.kI, timeout);
        srx.config_kD(slot, pid.kD, timeout);
        srx.config_kF(slot, pid.kFF, timeout);
        srx.config_IntegralZone(slot, (int) pid.kIz, timeout);
        srx.configAllowableClosedloopError(slot, 400, timeout);
        srx.configMaxIntegralAccumulator(slot, pid.maxIntegralAccumulation, timeout);
        srx.configClosedloopRamp(0);
        logf("Setup %s PID slot %d %s\n", pid.name, slot, pid.getPidData());
    }

    // This method will be called once per scheduler run
    @Override
    public void periodic() {

        // double left = RobotContainer.getLeftTrigger();
        // if (RobotContainer.getLeftBumper()) {
        //     left *= -1;
        // }
        // if (Math.abs(left) > .05) {
        //     setShoulderSpeed(left * .3);
        //     // logf("Shoulder Speed %.2f\n", left * .3);
        // } else {
        //     setShoulderSpeed(0);
        // }

        // double right = RobotContainer.getRightTrigger();
        // if (RobotContainer.getRightBumper()) {
        //     right *= -1;
        // }
        // if (Math.abs(right) > .05) {
        //     setExtenderSpeed(right * .3);
        //     // logf("Extender Speed %.2f\n", right * .3);
        // } else {
        //     setExtenderSpeed(0);
        // }

        // if (state == ZERO_ENCODER) {
        //     if (getReverseLimitSwitch(shoulderMotor)) {
        //         state = NORMAL;
        //         zeroEncoder(shoulderMotor);
        //         logf("Shoulder Homed\n");
        //         // setShoulderVelocity(0);
        //     }
        //     // setShoulderVelocity(0.3);
        //     return;
        // }

        
        // Display Shoulder Data
        if (Robot.count % 15 == 5) {
            double current = shoulderMotor.getStatorCurrent();
            SmartDashboard.putNumber("ShlCur", current);
            boolean forwardLimit = getForwardLimitSwitch(shoulderMotor);
            SmartDashboard.putBoolean("ShlForL", forwardLimit);
            boolean reverseLimit = getReverseLimitSwitch(shoulderMotor);
            SmartDashboard.putBoolean("ShlRevL", reverseLimit);
            double position = getShoulderPos();
            SmartDashboard.putNumber("ShlPos", position);
        }

        // Display Eextender Data
        if (Robot.count % 15 == 10) {
            double current = extenderMotor.getStatorCurrent();
            SmartDashboard.putNumber("ExtCur", current);
            boolean forwardLimit = getForwardLimitSwitch(extenderMotor);
            SmartDashboard.putBoolean("ExtForL", forwardLimit);
            boolean reverseLimit = getReverseLimitSwitch(extenderMotor);
            SmartDashboard.putBoolean("ExtRevL", reverseLimit);
            double position = getExtenderPos();
            SmartDashboard.putNumber("ExtPos", position);
        }

        // if (RobotContainer.mrKeith) {
        //     int pov = RobotContainer.getPov();
        //     if (lastPOV == pov) {
        //         return;
        //     }
        //     lastPOV = pov;
        //     if (pov == 0) {
        //         setMotorToPosition(shoulderMotor, 0);
        //     } else if (pov == 90) {
        //         setMotorToPosition(shoulderMotor, 5000);
        //     } else if (pov == 180) {
        //         setMotorToPosition(shoulderMotor, 10000);
        //     } else if (pov == 270) {
        //         setMotorToPosition(shoulderMotor, 15000);
        //     }
        // }
    }
}