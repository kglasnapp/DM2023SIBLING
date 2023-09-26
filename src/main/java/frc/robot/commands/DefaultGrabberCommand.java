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
    boolean powerTilt = false;

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
        boolean left = operatorController.getHID().getRawButton(5);
        boolean right = operatorController.getHID().getRawButton(6);

        if (left && grabberSubsystem.isReady()) {
            intakeSubsystem.intakeIn();
        } else if (right && grabberSubsystem.isReady()) {
            intakeSubsystem.intakeOut();
        } else if (!(right || left)) {
            intakeSubsystem.intakeOff();
        }
        // if (!grabberSubsystem.isReady()) {
        //     return;
        // }

        int pov = RobotContainer.getDriverPov();
        if (pov != lastPov) {
            double angle = grabberSubsystem.getLastTiltAngle();
            if (pov == 180) {
                grabberSubsystem.setTiltAngle(angle + 5);
            }
            if (pov == 0) {
                grabberSubsystem.setTiltAngle(angle - 5);
            }
            lastPov = pov;
        } 

        if (RobotContainer.getRightTrigger() > .2){
            grabberSubsystem.setPower(RobotContainer.getRightTrigger()/2);
            powerTilt = true;
        } else if (RobotContainer.getLeftTrigger() > .2){
            grabberSubsystem.setPower(-RobotContainer.getLeftTrigger()/2);
            powerTilt = true;
        } else if (powerTilt){
            grabberSubsystem.setPower(0);
            powerTilt = false;
        }
    }
}
