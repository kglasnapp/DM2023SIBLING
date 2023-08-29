package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot;
import frc.robot.subsystems.GrabberSubsystem;

import static frc.robot.utilities.Util.logf;

public class GrabberDefaultCommand extends CommandBase {
    static GrabberSubsystem grabberSubsystem;
    CommandXboxController controller2;

    enum STATE {
        IDLE, HOMEING, RAISED, DROPED
    }

    STATE state = STATE.IDLE;

    public GrabberDefaultCommand(GrabberSubsystem rearGrabberSubsystem, CommandXboxController controller2) {
        GrabberDefaultCommand.grabberSubsystem = rearGrabberSubsystem;
        this.controller2 = controller2;
        addRequirements(rearGrabberSubsystem);
    }

    @Override
    public void initialize() {
        grabberSubsystem.setTiltAngle(0);
        grabberSubsystem.setIntakePower(0);
        state = STATE.RAISED;
        logf("Init Rear Grab Default %d\n", Robot.count);
    }

    @Override
    public void execute() {
        double joyX = controller2.getLeftX();
        double joyY = controller2.getLeftY();
        if (Robot.count % 50 == 0) {
            // logf("Rear Grabber State:%s joyX:%.2f joyY:%.2f Tilt Pos:%.2f Last:%.2f Cur:%.2f\n", GrabberDefaultCommand.state, joyX, joyY,
            //         rearGrabberSubsystem.getTiltPos(), rearGrabberSubsystem.getLastTiltPos(), rearGrabberSubsystem.getTiltCurrent());
        }
        if (joyX < -0.8) {
            grabberSubsystem.setIntakePower(-1);
        } else if (joyX > 0.8) {
            grabberSubsystem.setIntakePower(1);
        } else {
            grabberSubsystem.setIntakePower(0);
        }
        double angle = grabberSubsystem.getLastTiltAngle();
        if (joyY > .8) {
            grabberSubsystem.setTiltAngle(angle + 5);
        }
        if (joyY < -0.8) {
            grabberSubsystem.setTiltAngle(angle - 5);
        }
    }

}
