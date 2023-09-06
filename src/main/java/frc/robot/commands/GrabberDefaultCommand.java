package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.GrabberTiltSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

import static frc.robot.utilities.Util.logf;

public class GrabberDefaultCommand extends CommandBase {
    static GrabberTiltSubsystem grabberSubsystem;
    IntakeSubsystem intakeSubsystem;
    CommandXboxController controller2;
    int lastPov = -1;

    enum STATE {
        IDLE, HOMEING, RAISED, DROPED
    }

    STATE state = STATE.IDLE;

    public GrabberDefaultCommand(GrabberTiltSubsystem grabberSubsystem, IntakeSubsystem intakeSubsystem, CommandXboxController operatorController) {
        GrabberDefaultCommand.grabberSubsystem = grabberSubsystem;
        this.controller2 = operatorController;
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(grabberSubsystem);
    }

    @Override
    public void initialize() {
        grabberSubsystem.setTiltAngle(0);
       intakeSubsystem.intakeOff();
        logf("Init Rear Grab Default %d\n", Robot.count);
    }

    @Override
    public void execute() {
        int pov = RobotContainer.getDriverPov();
        if (pov != lastPov) {
            logf("Pov: %d\n", pov);
            if (pov == 270) {
                intakeSubsystem.intakeIn();
            } else if (pov == 90) {
               intakeSubsystem.intakeOut();;
            } else if (pov == -1) {
                intakeSubsystem.intakeOff();;
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
