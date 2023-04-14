package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot;
import frc.robot.subsystems.RearGrabberSubsystem;

import static frc.robot.utilities.Util.logf;

public class RearGrabberDefaultCommand extends CommandBase {
    static RearGrabberSubsystem rearGrabberSubsystem;
    CommandXboxController controller2;

    enum STATE {
        IDLE, HOMEING, RAISED, DROPED
    }

    STATE state = STATE.IDLE;

    public RearGrabberDefaultCommand(RearGrabberSubsystem rearGrabberSubsystem, CommandXboxController controller2) {
        RearGrabberDefaultCommand.rearGrabberSubsystem = rearGrabberSubsystem;
        this.controller2 = controller2;
        addRequirements(rearGrabberSubsystem);
    }

    @Override
    public void initialize() {
        // rearGrabberSubsystem.setTiltPosition(200);
        rearGrabberSubsystem.setIntakePower(0);
        state = STATE.RAISED;
        logf("Init Rear Grab Default %d\n", Robot.count);
    }

    @Override
    public void execute() {
        double joyX = controller2.getLeftX();
        double joyY = controller2.getLeftY();
        if (Robot.count % 50 == 0) {
            logf("Rear Grabber State:%s joyX:%.2f joyY:%.2f Tilt Pos:%.2f\n", GrabberDefaultCommand.state, joyX, joyY,
                    rearGrabberSubsystem.getTiltPos());
        }
        if (joyX < -0.8) {
            rearGrabberSubsystem.setIntakePower(-1);
        } else if (joyX > 0.8) {
            rearGrabberSubsystem.setIntakePower(1);
        } else {
            rearGrabberSubsystem.setIntakePower(0);
        }

        if (joyY < -0.8) {
            // rearGrabberSubsystem.setTiltPosition(-200);
        }
        if (joyY > .8) {
            // rearGrabberSubsystem.setTiltPosition(-20);
        }
    }

}
