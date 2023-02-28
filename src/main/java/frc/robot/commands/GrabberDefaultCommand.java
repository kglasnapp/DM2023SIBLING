package frc.robot.commands;

import edu.wpi.first.wpilibj.RobotController;
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

    double CURRENT_THRESHOLD = 6.5;
    RunningAverage avg = new RunningAverage(20);

    BooleanSupplier openProvider;
    BooleanSupplier closeProvider;
    BooleanSupplier stopProvider;
    BooleanSupplier openProvider2;
    BooleanSupplier closeProvider2;
    BooleanSupplier stopProvider2;

    public enum State {
        IDLE, START_HOME_GRABBER, HOMING_GRABBER, READY, OVERCURRENT
    }

    static State state = State.START_HOME_GRABBER;

    public GrabberDefaultCommand(GrabberSubsystem grabberSubsystem,
            BooleanSupplier openProvider,
            BooleanSupplier closeProvider,
            BooleanSupplier stopProvider,
            BooleanSupplier openProvider2,
            BooleanSupplier closeProvider2,
            BooleanSupplier stopProvider2) {
        this.grabberSubsystem = grabberSubsystem;
        this.openProvider = openProvider;
        this.closeProvider = closeProvider;
        this.stopProvider = stopProvider;
        this.openProvider2 = openProvider2;
        this.closeProvider2 = closeProvider2;
        this.stopProvider2 = stopProvider2;
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
                logf("Grabber Homed %.2f\n", avgCurrent);
            }
        }
        if (grabberSubsystem.getForwardLimitSwitch()) {
            grabberSubsystem.zeroEncoder();
        }

        double presentPowerLevel = grabberSubsystem.getLastPowerLevel();
        if (Robot.count % 4 == 2) {
            SmartDashboard.putString("Gab State", state.toString());
            SmartDashboard.putNumber("Avg Cur", avgCurrent);
            SmartDashboard.putNumber("Pres PL", presentPowerLevel);
        }
        if (state == State.READY) {
            //double currentTime = RobotController.getFPGATime() / 1000;
            if (avgCurrent > CURRENT_THRESHOLD) { //} && currentTime - GrabberSubsystem.startGrab > 20) {
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

        if (openProvider.getAsBoolean() || openProvider2.getAsBoolean()) {
            grabberSubsystem.setGrabberPower(.8);
        }
        if (closeProvider.getAsBoolean() || closeProvider2.getAsBoolean()) {
            grabberSubsystem.setGrabberPower(-.8);
        }
        if (stopProvider.getAsBoolean() || stopProvider2.getAsBoolean()) {
            grabberSubsystem.setGrabberPower(0);
        }
    }
}
