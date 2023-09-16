package frc.robot.commands;

import static frc.robot.Util.logf;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GrabberTiltSubsystem;

public class DefaultElevatorCommand extends CommandBase {
    ElevatorSubsystem elevatorSubsystem;
    GrabberTiltSubsystem grabberSubsystem;
    CommandXboxController operatorController;
    int lastPov = -1;
    final private double MAX_CURRENT = 20;
    private int myCount = 0;

    enum STATE {
        IDLE, HOMING, READY, OVERCURRENT, OVERCURRENTSTOPPED
    }

    STATE state = STATE.IDLE;
    STATE lastState = null;

    public DefaultElevatorCommand(ElevatorSubsystem elevatorSubsystem, GrabberTiltSubsystem grabberSubsystem,
            CommandXboxController operatorController) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.grabberSubsystem = grabberSubsystem;
        this.operatorController = operatorController;

        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
        logf("Init Elevator Default Command\n");
    }

    @Override
    public void execute() {
        double current = elevatorSubsystem.getElevatorCurrent();
        if (state != lastState) {
            logf("Elevator State Changed state:%s current:%.3f myCount:%d angle:%.2f\n", state, current, myCount,
                    grabberSubsystem.getAbsEncoder());
            lastState = state;
        }
        switch (state) {
            case IDLE:
                // Can home only if intake is retracted
                if (!grabberSubsystem.retracted()) {
                    if (Robot.count % 50 == 12) {
                        logf("Can't Home since the grabber is not retracted angle:%.3f\n",
                                grabberSubsystem.getAbsEncoder());
                    }
                    break;
                }
                state = STATE.HOMING;
                break;
            case HOMING:
                if (elevatorSubsystem.getReverseLimitSwitch()) {
                    // At home so stop motor and indicate homed
                    elevatorSubsystem.setPower(0);
                    elevatorSubsystem.setHomed(true);
                    elevatorSubsystem.setEncoder(0);
                    state = STATE.READY;
                    logf("Elevator is homed\n");
                    break;
                }
                if (current > MAX_CURRENT) {
                    logf("Elevator overcurrent detected while homing current:%.2f\n", current);
                    myCount = 5; // Wait 100 ms to see if over current remains
                    state = STATE.OVERCURRENT;
                    break;
                }
                elevatorSubsystem.setPower(-.2);
                break;
            case READY:
                if (current > MAX_CURRENT) {
                    logf("Elevator overcurrent detected while ready current:%.2f\n", current);
                    myCount = 5; // Wait 100 ms to see if over current remains
                    state = STATE.OVERCURRENT;
                    break;
                }
                boolean left = operatorController.getHID().getRawButtonPressed(5);
                boolean right = operatorController.getHID().getRawButtonPressed(6);
                double pos = elevatorSubsystem.getLastElevatorPositionInches();
                if (left) {
                    elevatorSubsystem.setElevatorPos(pos + 3);
                }
                if (right) {
                    elevatorSubsystem.setElevatorPos(pos - 3);
                }
                break;
            case OVERCURRENT:
                myCount--;
                if (myCount < 0) {
                    elevatorSubsystem.setPower(0);
                    myCount = 20; // Wait 400 ms to restart
                    state = STATE.OVERCURRENTSTOPPED;
                }
            case OVERCURRENTSTOPPED:
                myCount--;
                if (myCount < 0) {
                    if (current > MAX_CURRENT) {
                        // If curent remains high continue to wait
                        logf("Elevator current remains high -- current:%.2f\n", current);
                        myCount = 20; // Wait another 400 ms for current to go low
                        break;
                    }
                    // Current seems to have stablize restore last task
                    if (elevatorSubsystem.getHomed()) {
                        // TODO Restore any running PID
                    } else {
                        // Try to home again
                        state = STATE.IDLE;
                    }
                }
        }
    }
}
