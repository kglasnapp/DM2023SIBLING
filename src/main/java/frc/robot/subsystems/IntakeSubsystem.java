package frc.robot.subsystems;

import static frc.robot.Util.logf;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.RobotMode;
import frc.robot.subsystems.LedSubsystem.Leds;
import com.revrobotics.SparkMaxPIDController;

public class IntakeSubsystem extends SubsystemBase {
    private static final int GRABBER_INTAKE_MOTOR_ID = 10;
    private double lastIntakePower = 0;
    private final CANSparkMax intakeMotor;
    private final double defaultIntakePower = .3;
    private final double overCurrentPower = .05;
    private final double maxCurrent = 8;
    private final double maxCurrentLow = 2 ;
    private PID_MAX pid = new PID_MAX();
    private int timeOverMax = 0;
    private int timeAtOverCurrent = 0;
    private SparkMaxPIDController pidController;
    private boolean currentMode = true;

    public IntakeSubsystem() {
        // Setup parameters for the intake motor
        intakeMotor = new CANSparkMax(GRABBER_INTAKE_MOTOR_ID, MotorType.kBrushless);
        intakeMotor.restoreFactoryDefaults();
        intakeMotor.setSmartCurrentLimit(8);
        intakeOff(); 
        pidController = intakeMotor.getPIDController();
        RobotContainer.leds.setOverCurrent(Leds.IntakeOverCurrent, false);
        pid.PIDCoefficientsIntake(pidController);
        pid.PIDToMax();
        // pid.putPidCoefficientToDashBoard();
    }

    public void setBrakeMode(boolean mode) {
        intakeMotor.setIdleMode(mode ? IdleMode.kBrake : IdleMode.kCoast);
        logf("Brake mode: %s\n", intakeMotor.getIdleMode());
    }

    public void setIntakeCurrent(double current) {
        pidController.setReference(current, CANSparkMax.ControlType.kCurrent);
    }

    public void intakeIn() {
        if (RobotContainer.robotMode == RobotMode.Cone) {
            setIntakePower(defaultIntakePower);
        } else {
            setIntakePower(-defaultIntakePower);
        }
    }

    public void intakeOut() {
        if (RobotContainer.robotMode == RobotMode.Cone) {
            setIntakePower(-defaultIntakePower);
        } else {
            setIntakePower(defaultIntakePower);
        }
    }

    public void intakeOff() {
        setIntakePower(0);
    }

    private void setIntakePower(double power) {
        if (lastIntakePower != power) {
            lastIntakePower = power;
            if (currentMode) {
                logf("Grabber Intakecurrent:%.2f\n", power * 30);
                setIntakeCurrent(power * 30);
                //timeOverMax = 0;
            } else {
                intakeMotor.set(power);
                logf("Grabber Intake power:%.2f\n", power);
                //timeOverMax = 0;
            }
        }
    }

    private void setReducedIntakePower() {
        if (lastIntakePower != 0.0) {
            if (RobotContainer.robotMode == RobotMode.Cone) {
                setIntakePower(-overCurrentPower);
            } else {
                setIntakePower(overCurrentPower);
            }
        } else {
            setIntakePower(0.0);
        }
    }

    // This method will be called once per scheduler run
    @Override
    public void periodic() {
        double current = intakeMotor.getOutputCurrent();
        if (timeAtOverCurrent == 0) {
            if (current > maxCurrent) {
                timeOverMax++;
                if (timeOverMax > 8) {
                    logf("Intake Over Current %,2f\n", current);
                    setReducedIntakePower();
                    RobotContainer.leds.setOverCurrent(Leds.IntakeOverCurrent, true);
                    timeAtOverCurrent = 15;
                    timeOverMax = 0;
                }
            }
        } else {
            timeAtOverCurrent--;
            if (current > maxCurrentLow) {
                timeAtOverCurrent = 15;
            }
            if (timeAtOverCurrent <= 0) {
                RobotContainer.leds.setOverCurrent(Leds.IntakeOverCurrent, false);
                intakeMotor.set(lastIntakePower);
                timeAtOverCurrent = 0;
            } else {
                setReducedIntakePower();
            }
        }

        if (Robot.count % 20 == 6) {
            SmartDashboard.putNumber("Intk Cur", current);
            SmartDashboard.putNumber("Intk Pwr", lastIntakePower);
        }
    }

    enum STATE {
        NORMAL, WAITOVER, INOVERCURRENT,
    }

    class currentControl {
        int overMaxCnt;
        double maxCurrent;
        int atOverCurrentCnt;
        double reducedCurrent;
        int myCount;
        STATE state;

        public currentControl(double maxCurrent, int overMaxCnt, double reducedCurrent, int atOverCurrentCnt) {
            this.maxCurrent = maxCurrent;
            this.overMaxCnt = overMaxCnt;
            this.reducedCurrent = reducedCurrent;
            this.atOverCurrentCnt = atOverCurrentCnt;
            myCount = 0;
            state = STATE.NORMAL;
        }

        boolean periodic(double current) {
            switch (state) {
                case NORMAL:
                    if (current > maxCurrent) {
                        myCount = overMaxCnt;
                        state = STATE.WAITOVER;
                    }
                    return true;
                case WAITOVER:
                    myCount--;
                    if (myCount < 0) {
                        if (current > maxCurrent) {
                            state = STATE.INOVERCURRENT;
                            myCount = atOverCurrentCnt;
                        }
                    }
                    return false;
                case INOVERCURRENT:
                    myCount--;
                    if (myCount < 0) {
                        if (current < reducedCurrent) {
                            state = STATE.NORMAL;
                            return true;
                        }
                        myCount = atOverCurrentCnt;
                    }
                    return false;
            }
            return false;
        }
    }
}
