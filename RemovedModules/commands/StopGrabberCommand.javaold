package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GrabberSubsystem;

public class StopGrabberCommand extends CommandBase {
    GrabberSubsystem grabberSubsystem;

    public StopGrabberCommand(GrabberSubsystem grabberSubsystem) {
        this.grabberSubsystem = grabberSubsystem;
        
        //addRequirements(grabberSubsystem);
    }

    @Override
    public void initialize() {
        double speed = 0;
        grabberSubsystem.setGrabberPower(speed);
        GrabberDefaultCommand.state = GrabberDefaultCommand.State.READY;
    }

    @Override
    public void execute() {
        double speed = 0;
        grabberSubsystem.setGrabberPower(speed);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}


