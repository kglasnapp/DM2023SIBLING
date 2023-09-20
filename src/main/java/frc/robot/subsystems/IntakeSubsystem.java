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
import frc.robot.utilities.RunningAverage;

import com.revrobotics.SparkMaxPIDController;

public class IntakeSubsystem extends SubsystemBase {
    private static final int GRABBER_INTAKE_MOTOR_ID = 10;
    private double targetIntakePower = 0;
    private double lastIntakePower = 0;
    private final CANSparkMax intakeMotor;

    private final double defaultIntakePowerInCone = .6;
    private final double defaultIntakePowerOutCone = .6;
    private final double defaultIntakePowerInCube = .6;
    private final double defaultIntakePowerOutCube = 1.0;

    private final double overCurrentPower = .05;

    private final double maxCurrentCone = 10;
    private final double maxCurrentLowCone = 2;
    private final double maxCurrentCube = 5;
    private final double maxCurrentLowCube = 2;

    //private PID_MAX pid = new PID_MAX();
    //private int timeOverMax = 0;
    //private int timeAtOverCurrent = 0;
    private SparkMaxPIDController pidController;
    private RunningAverage avg = new RunningAverage(10);

    public IntakeSubsystem() {
        // Setup parameters for the intake motor
        intakeMotor = new CANSparkMax(GRABBER_INTAKE_MOTOR_ID, MotorType.kBrushless);
        intakeMotor.restoreFactoryDefaults();
        intakeMotor.setSmartCurrentLimit(15);
        intakeOff();
        setBrakeMode(true);
        RobotContainer.leds.setOverCurrent(Leds.IntakeOverCurrent, false);

        // The following are needed if running in current mode
        //pidController = intakeMotor.getPIDController();
        //pid.PIDCoefficientsIntake(pidController);
        //pid.PIDToMax();
        // pid.putPidCoefficientToDashBoard();
    }

    public void setBrakeMode(boolean mode) {
        intakeMotor.setIdleMode(mode ? IdleMode.kBrake : IdleMode.kCoast);
        logf("Brake mode: %s\n", intakeMotor.getIdleMode());
    }

    public void intakeIn() {
        if (RobotContainer.robotMode == RobotMode.Cone) {
            setIntakePower(-defaultIntakePowerInCone);
        } else {
            setIntakePower(defaultIntakePowerInCube);
        }
    }

    public void intakeOut() {
        if (RobotContainer.robotMode == RobotMode.Cone) {
            setIntakePower(defaultIntakePowerOutCone);
        } else {
            setIntakePower(-defaultIntakePowerOutCube);
        }
    }

    public void intakeOff() {
        setIntakePower(0);
    }

    private void setIntakePower(double power) {
        targetIntakePower = power;
    }

    private double getReducedIntakePower() {
        if (RobotContainer.robotMode == RobotMode.Cone) {
            return overCurrentPower;
        } else {
            return -overCurrentPower;
        }
    }

    enum STATE {
        NORMAL, OVERCURRENT,
    }

    private STATE state = STATE.NORMAL;
    // cycles over current limit
    private int overcurrentCountUp = 0;
    // cycles to remain in overcurrent state
    private int overcurrentCountDown = 0;

    private static final int overcurrentCountUpLimit = 30;
    private static final int overcurrentCountDownLimit = 15;

    // This method will be called once per scheduler run
    @Override
    public void periodic() {
        double maxCurrent = RobotContainer.robotMode == RobotContainer.RobotMode.Cone ? maxCurrentCone : maxCurrentCube;
        double maxCurrentLow = RobotContainer.robotMode == RobotContainer.RobotMode.Cone ? maxCurrentLowCone
                : maxCurrentLowCube;

        double current = intakeMotor.getOutputCurrent();
        double avgCurrent = avg.add(current);
        double power = targetIntakePower;

        if (state == STATE.NORMAL) {
            if (avgCurrent > maxCurrent) {
                overcurrentCountUp++;
            }
            if (overcurrentCountUp >= overcurrentCountUpLimit) {
                state = STATE.OVERCURRENT;
                overcurrentCountDown = overcurrentCountDownLimit;
                overcurrentCountUp = 0;
                logf("Intake Overcurrent detected avg current:%.2f\n", avgCurrent);
                RobotContainer.leds.setOverCurrent(Leds.IntakeOverCurrent, true);
            }
        }
        if (state == STATE.OVERCURRENT) {
            overcurrentCountDown--;
            if (avgCurrent > maxCurrentLow) {
                overcurrentCountDown = overcurrentCountDownLimit;
            }

            if (overcurrentCountDown <= 0) {
                state = STATE.NORMAL;
                RobotContainer.leds.setOverCurrent(Leds.IntakeOverCurrent, false);
                overcurrentCountDown = 0;
            }

            power = getReducedIntakePower();
        }

        if (power != lastIntakePower) {
            intakeMotor.set(power);
            lastIntakePower = power;
        }

        if (Robot.count % 10 == 1) {
            SmartDashboard.putNumber("Intk Cur", current);
            SmartDashboard.putNumber("Intk ACur", avgCurrent);
            SmartDashboard.putNumber("Intk TPwr", lastIntakePower);
            SmartDashboard.putNumber("Intk Pwr", power);
        }
    }

    // enum STATE {
    //     NORMAL, WAITOVER, INOVERCURRENT,
    // }

    // class currentControl {
    //     int overMaxCnt;
    //     double maxCurrent;
    //     int atOverCurrentCnt;
    //     double reducedCurrent;
    //     int myCount;
    //     STATE state;

    //     public currentControl(double maxCurrent, int overMaxCnt, double reducedCurrent, int atOverCurrentCnt) {
    //         this.maxCurrent = maxCurrent;
    //         this.overMaxCnt = overMaxCnt;
    //         this.reducedCurrent = reducedCurrent;
    //         this.atOverCurrentCnt = atOverCurrentCnt;
    //         myCount = 0;
    //         state = STATE.NORMAL;
    //     }

    //     boolean periodic(double current) {
    //         switch (state) {
    //             case NORMAL:
    //                 if (current > maxCurrent) {
    //                     myCount = overMaxCnt;
    //                     state = STATE.WAITOVER;
    //                 }
    //                 return true;
    //             case WAITOVER:
    //                 myCount--;
    //                 if (myCount < 0) {
    //                     if (current > maxCurrent) {
    //                         state = STATE.INOVERCURRENT;
    //                         myCount = atOverCurrentCnt;
    //                     }
    //                 }
    //                 return false;
    //             case INOVERCURRENT:
    //                 myCount--;
    //                 if (myCount < 0) {
    //                     if (current < reducedCurrent) {
    //                         state = STATE.NORMAL;
    //                         return true;
    //                     }
    //                     myCount = atOverCurrentCnt;
    //                 }
    //                 return false;
    //         }
    //         return false;
    //     }
    // }
}
