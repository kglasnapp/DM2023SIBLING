package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.subsystems.GrabberTiltSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

import static frc.robot.utilities.Util.logf;

public class DefaultGrabberCommand extends CommandBase {
    static GrabberTiltSubsystem grabberSubsystem;
    IntakeSubsystem intakeSubsystem;
    CommandXboxController operatorController;
    int lastPov = -1;

    public DefaultGrabberCommand(GrabberTiltSubsystem grabberSubsystem, IntakeSubsystem intakeSubsystem,
            CommandXboxController operatorController) {
        DefaultGrabberCommand.grabberSubsystem = grabberSubsystem;
        this.operatorController = operatorController;
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(grabberSubsystem);

    }

    @Override
    public void initialize() {
        if (intakeSubsystem != null) {
            intakeSubsystem.intakeOff();
        }
        logf("Init Grabber Default Command\n");
    }

    @Override
    public void execute() {
        if (!grabberSubsystem.isReady()) {
            return;
        }
        boolean left = operatorController.getHID().getRawButton(5);
        boolean right = operatorController.getHID().getRawButton(6);
        if (left) {
            intakeSubsystem.intakeIn();
        }
        if (right) {
            intakeSubsystem.intakeOut();
        }
        if (!(right || left)) {
            intakeSubsystem.intakeOff();
        }
        int pov = RobotContainer.getDriverPov();
        if (pov != lastPov) {
            double angle = grabberSubsystem.getLastTiltAngle();
            if (pov == 180) {
                grabberSubsystem.setTiltAngle(angle + 20);
            }
            if (pov == 0) {
                grabberSubsystem.setTiltAngle(angle - 20);
            }
            lastPov = pov;
        }
    }
}
