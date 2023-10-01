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

public class IntakeSubsystem extends SubsystemBase {
    private static final int GRABBER_INTAKE_MOTOR_ID = 10;
    private double targetIntakePower = 0;
    private double lastIntakePower = 0;
    private final CANSparkMax intakeMotor;

    private final double defaultIntakePowerInCone = .6;
    private final double defaultIntakePowerOutCone = .6;
    private final double defaultIntakePowerInCube = .8;
    private final double defaultIntakePowerOutCube = 1.0;;
    private RunningAverage avg = new RunningAverage(5);

    private boolean isOut = false;

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
        isOut = false;
        if (RobotContainer.robotMode == RobotMode.Cone) {
            setIntakePower(-defaultIntakePowerInCone);
        } else {
            setIntakePower(defaultIntakePowerInCube);
        }
    }

    public void intakeOut() {
        isOut = true;
        if (RobotContainer.robotMode == RobotMode.Cone) {
            setIntakePower(defaultIntakePowerOutCone);
        } else {
            setIntakePower(-defaultIntakePowerOutCube);
        }
    }

    public void intakeOff() {
        isOut = false;
        setIntakePower(0);
    }

    public boolean overCurrent() {
        return StateMachineForCurrent.overCurrent();
    }

    private void setIntakePower(double power) {
        targetIntakePower = power;
    }

    // This method will be called once per scheduler run
    @Override
    public void periodic() {
        double current = intakeMotor.getOutputCurrent();
        double avgCurrent = avg.add(current);
        if (current > .05) {
            logf("Cur:%.2f Avg:%.2f Count:%d\n", current, avgCurrent, StateMachineForCurrent.counter);
        }
        double power;
        if (!isOut) {
            power = StateMachineForCurrent.periodic(targetIntakePower, avgCurrent);
        } else {
            power = targetIntakePower;
        }
        if (power != lastIntakePower) {
            intakeMotor.set(power);
            lastIntakePower = power;
        }
        if (Robot.count % 10 == 1) {
            SmartDashboard.putNumber("Intk Cur", current);
            SmartDashboard.putNumber("Intk ACur", avgCurrent);
            //SmartDashboard.putNumber("Intk TPwr", lastIntakePower);
            SmartDashboard.putNumber("Intk Pwr", power);
        }
    }

    static class StateMachineForCurrent {
        private static final double maxCurrentCone = 20;
        private static final double maxCurrentLowCone = 2;
        private static final double maxCurrentCube = 6;
        private static final double maxCurrentLowCube = 2;
        private static final int OVERCURRENT_COUNT_LIMIT = 10;
        private static final int BACK_TO_NORMAL_COUNT_LIMIT = 15;
        private static final double overCurrentPower = .1;

        static int counter = 0;

        enum STATE {
            NORMAL, OVERCURRENT,
        }

        static STATE state = STATE.NORMAL;

        public static double periodic(double power, double avgCurrent) {
            boolean coneMode = RobotContainer.robotMode == RobotContainer.RobotMode.Cone;
            double maxCurrent = coneMode ? maxCurrentCone : maxCurrentCube;
            double maxCurrentLow = coneMode ? maxCurrentLowCone : maxCurrentLowCube;
            if (state == STATE.NORMAL) {
                if (avgCurrent > maxCurrent) {
                    counter++;
                } else {
                    counter--;

                    if (counter < 0) {
                        counter = 0;
                    }
                }
                if (counter >= OVERCURRENT_COUNT_LIMIT) {
                    state = STATE.OVERCURRENT;
                    counter = 0;
                    logf("Intake Overcurrent detected avg current:%.2f\n", avgCurrent);
                    RobotContainer.leds.setOverCurrent(Leds.IntakeOverCurrent, true);
                }
            }
            if (state == STATE.OVERCURRENT) {
                if (power == 0.0) {
                    state = STATE.NORMAL;
                    counter = 0;
                    return 0.0;
                }
                if (avgCurrent > maxCurrentLow) {
                    counter = 0;
                }
                if (counter >= BACK_TO_NORMAL_COUNT_LIMIT) {
                    state = STATE.NORMAL;
                    RobotContainer.leds.setOverCurrent(Leds.IntakeOverCurrent, false);
                    counter = 0;
                }
                power = Math.signum(power) * overCurrentPower;
                counter++;
            }
            return power;
        }

        public static boolean overCurrent() {
            return state == STATE.OVERCURRENT && counter > BACK_TO_NORMAL_COUNT_LIMIT - 2;
        }
    }
}
