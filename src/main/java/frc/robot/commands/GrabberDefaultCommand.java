package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.utilities.RunningAverage;

import static frc.robot.utilities.Util.logf;

import java.util.function.BooleanSupplier;

public class GrabberDefaultCommand extends CommandBase {

    GrabberSubsystem grabberSubsystem;
    double lastPowerLevel = 0;

    double CURRENT_THRESHOLD = 5;
    RunningAverage avg = new RunningAverage(20);

    BooleanSupplier openProvider;
    BooleanSupplier closeProvider;
    BooleanSupplier stopProvider;

    public enum State {
        IDLE, START_HOME_GRABBER, HOMING_GRABBER, READY, OVERCURRENT
    }

    static State state = State.START_HOME_GRABBER;

    public GrabberDefaultCommand(GrabberSubsystem grabberSubsystem,
            BooleanSupplier openProvider,
            BooleanSupplier closeProvider,
            BooleanSupplier stopProvider) {
        this.grabberSubsystem = grabberSubsystem;
        this.openProvider = openProvider;
        this.closeProvider = closeProvider;
        this.stopProvider = stopProvider;
        addRequirements(grabberSubsystem);
    }

    @Override
    public void initialize() {
        grabberSubsystem.lastGrabberStopPosition = grabberSubsystem.getGrabberPos();
        logf("Default Grabber Init\n");
        avg.init();
        state = State.START_HOME_GRABBER;
    }

    @Override
    public void execute() {
        double avgCurrent = Math.abs(avg.add(grabberSubsystem.getStatorCurrent()));
        if (state == State.START_HOME_GRABBER) {
            grabberSubsystem.setGrabberPower(0.6);
            state = State.HOMING_GRABBER;
        }
        if (state == State.HOMING_GRABBER) {
            if (avgCurrent > CURRENT_THRESHOLD) {
                state = State.READY;
                grabberSubsystem.zeroEncoder();
                grabberSubsystem.setGrabberPower(0);
                logf("Grabber Homed\n");
            }
        }
        if (grabberSubsystem.getForwardLimitSwitch()) {
            grabberSubsystem.zeroEncoder();
        }

        double presentPowerLevel = grabberSubsystem.getLastPowerLevel();
        if (Robot.count % 15 == 7) {
            SmartDashboard.putString("Gab State", state.toString());
            SmartDashboard.putNumber("Avg Cur", avgCurrent);
            SmartDashboard.putNumber("Pres PL", presentPowerLevel);
        }
        if (state == State.READY) {
            if (avgCurrent > CURRENT_THRESHOLD) {
                logf("Current %.2f found to be large\n", avgCurrent);
                state = State.OVERCURRENT;
                lastPowerLevel = grabberSubsystem.getLastPowerLevel();
                grabberSubsystem.setGrabberPower(0);
            }
        }
        if (state == State.OVERCURRENT) {
            if (Math.abs(presentPowerLevel) > 0 && avgCurrent < 2.0) {
                grabberSubsystem.setGrabberPower(presentPowerLevel);
                state = State.READY;
                logf("Set New Power Level  %.2f after over current lastest %.2f\n", presentPowerLevel, avgCurrent);
            }

        }

        if (openProvider.getAsBoolean()) {
            grabberSubsystem.setGrabberPower(.6);
        }
        if (closeProvider.getAsBoolean()) {
            grabberSubsystem.setGrabberPower(-.6);
        }
        if (stopProvider.getAsBoolean()) {
            grabberSubsystem.setGrabberPower(0);
        }
    }
}
