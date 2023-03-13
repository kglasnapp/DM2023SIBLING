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

    static GrabberSubsystem grabberSubsystem;
    double lastPowerLevel = 0;

    double CURRENT_THRESHOLD = 6.5;
    RunningAverage avg = new RunningAverage(20);
    
    static double timer = 0;

    BooleanSupplier openProvider;
    BooleanSupplier closeProvider;
    BooleanSupplier stopProvider;
    BooleanSupplier openProvider2;
    BooleanSupplier closeProvider2;
    BooleanSupplier stopProvider2;

    public static enum State {
        IDLE, START_HOME_GRABBER, HOMING_GRABBER, READY, OVERCURRENT, DROPPING
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
        state = State.READY;
    }
    
    

    @Override
    public void execute() {
        double avgCurrent = Math.abs(avg.add(grabberSubsystem.getStatorCurrent()));
        
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
            if (avgCurrent > CURRENT_THRESHOLD) {
                timer = RobotController.getFPGATime() / 1000;
                state = State.OVERCURRENT;
            }
        }
        if (state == State.OVERCURRENT) {
           if (timer + 3000 < RobotController.getFPGATime() / 1000) {
                grabberSubsystem.setGrabberPower(0);
                state = State.READY;
           }
        }

        if (state == State.DROPPING) {
            System.out.println("dropping");
            if (timer + 4000 < RobotController.getFPGATime() / 1000) {
                grabberSubsystem.setGrabberPower(0);
                state = State.READY;
           }
        }

        if (openProvider.getAsBoolean() || openProvider2.getAsBoolean()) {
            grabberSubsystem.setGrabberPower(.8);
        }
        if (closeProvider.getAsBoolean() || closeProvider2.getAsBoolean()) {
            startDropping();
        }
        if (stopProvider.getAsBoolean() || stopProvider2.getAsBoolean()) {
            grabberSubsystem.setGrabberPower(0);
        }
    }

    public static void startDropping() {
        grabberSubsystem.setGrabberPower(-.8);
        timer = RobotController.getFPGATime() / 1000;
        state = State.DROPPING;
    }
}
