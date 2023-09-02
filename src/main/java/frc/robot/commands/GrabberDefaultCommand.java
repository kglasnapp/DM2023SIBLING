package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.GrabberSubsystem;

import static frc.robot.utilities.Util.logf;

public class GrabberDefaultCommand extends CommandBase {
    static GrabberSubsystem grabberSubsystem;
    CommandXboxController controller2;
    int lastPov = -1;

    enum STATE {
        IDLE, HOMEING, RAISED, DROPED
    }

    STATE state = STATE.IDLE;

    public GrabberDefaultCommand(GrabberSubsystem rearGrabberSubsystem, CommandXboxController operatorController) {
        GrabberDefaultCommand.grabberSubsystem = rearGrabberSubsystem;
        this.controller2 = operatorController;
        addRequirements(rearGrabberSubsystem);
    }

    @Override
    public void initialize() {
        grabberSubsystem.setTiltAngle(0);
        grabberSubsystem.setIntakePower(0);
        logf("Init Rear Grab Default %d\n", Robot.count);
    }

    @Override
    public void execute() {
        int pov = RobotContainer.getPov();
        if (pov != lastPov) {
            logf("Pov: %d\n", pov);
            if (pov == 270) {
                grabberSubsystem.setIntakePower(-1);
            } else if (pov == 90) {
                grabberSubsystem.setIntakePower(1);
            } else if (pov == -1) {
                grabberSubsystem.setIntakePower(0);
            }
            double angle = grabberSubsystem.getLastTiltAngle();
            if (pov == 0) {
                grabberSubsystem.setTiltAngle(angle + 5);
            }
            if (pov == 180) {
                grabberSubsystem.setTiltAngle(angle - 5);
            }
            lastPov = pov;
        }
    }
}
