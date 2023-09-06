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

public class IntakeSubsystem extends SubsystemBase {
    private static final int GRABBER_INTAKE_MOTOR_ID = 10;
    private double lastIntakePower = 0;
    private final CANSparkMax intakeMotor;
    private final double defaultIntakePower = .3;
    private final double overCurrentPower = .1;
    private final double maxCurrent = 8;
    private int timeOverMax = 0;
    private int timeAtOverCurrent = 0;

    public IntakeSubsystem() {
        // Setup parameters for the intake motor
        intakeMotor = new CANSparkMax(GRABBER_INTAKE_MOTOR_ID, MotorType.kBrushless);
        intakeMotor.restoreFactoryDefaults();
        // TODO  Limit currnet for Testing
        intakeMotor.setSmartCurrentLimit(5);
        intakeOff();
    }

    public void setBrakeMode(boolean mode) {
        intakeMotor.setIdleMode(mode ? IdleMode.kBrake : IdleMode.kCoast);
        logf("Brake mode: %s\n", intakeMotor.getIdleMode());
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
            intakeMotor.set(power);
            logf("Grabber Intake power:%.2f\n", power);
            lastIntakePower = power;
            timeOverMax = 0;
        }
    }

    private void setReducedIntakePower() {
        intakeMotor.set(overCurrentPower);
    }
    // This method will be called once per scheduler run
    @Override
    public void periodic() {
        double current = intakeMotor.getOutputCurrent();
        if (current > maxCurrent) {
            timeOverMax++;
            if(timeOverMax > 4) {
                setReducedIntakePower();
                timeAtOverCurrent = 15;
            }
        }
        if(timeAtOverCurrent > 0) {
            timeAtOverCurrent--;
        } else {
            setIntakePower(lastIntakePower);
        }
        if (Robot.count % 20 == 6) {
            SmartDashboard.putNumber("Intk Cur", current);
            SmartDashboard.putNumber("Intk Pwr", lastIntakePower);
        }
    }

}
