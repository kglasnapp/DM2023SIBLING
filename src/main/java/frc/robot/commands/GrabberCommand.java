package frc.robot.commands;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GrabberSubsystem;
import static frc.robot.utilities.Util.logf;

public class GrabberCommand extends CommandBase {
    GrabberSubsystem grabberSubsystem;
    boolean open;

    public GrabberCommand(GrabberSubsystem grabberSubsystem, boolean open) {
        this.grabberSubsystem = grabberSubsystem;
        this.open = open;
        //addRequirements(grabberSubsystem);
    }

    @Override
    public void initialize() {
        GrabberSubsystem.startGrab = RobotController.getFPGATime() / 1000;
    }

    @Override
    public void execute() {
        double speed = 0.6;
        if (open) {
            speed = -0.6;
        } 
        logf("grabber command: %b state = %s\n", open, GrabberDefaultCommand.state);        
        grabberSubsystem.setGrabberPower(speed);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
