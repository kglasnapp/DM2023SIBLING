package frc.robot.commands;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

import static frc.robot.utilities.Util.logf;

public class IntakeCommand extends CommandBase {
    IntakeSubsystem intakeSubsystem;
    State state;
    double timeOut;
    double startTime;

    public enum State {
        IN, OUT, OFF
    }

    // This command sets the state of the intake to the value of the state
    // Turn off intake when timeout or over current hits
    public IntakeCommand(IntakeSubsystem intakeSubsystem, State state, double timeOut) {
        this.intakeSubsystem = intakeSubsystem;
        this.state = state;
        this.timeOut = timeOut;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        startTime = RobotController.getFPGATime();
        if (state == State.OUT) {
            intakeSubsystem.intakeOut();
        }
        if (state == State.IN) {
            intakeSubsystem.intakeIn();
        }
        if (state == State.OFF) {
            intakeSubsystem.intakeOff();
        }
        logf("Intake %s for %.2f counts\n", state, timeOut);
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        if (intakeSubsystem.overCurrent()) {
            logf("Intake Stopped due to overCurrent\n");
            return true;
        }
        // Check to see if command has run too long
        if (RobotController.getFPGATime()  > startTime + timeOut) {
            intakeSubsystem.intakeOff();
            return true;
        }
        return false;
    }
}
