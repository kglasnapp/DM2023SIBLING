package frc.robot.commands;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.OperatorButtons;
import static frc.robot.Util.logf;

public class PositionCommand extends CommandBase {
    /** Creates a new ReplaceMeCommand. */
    OperatorButtons type;
    int timeOut;
    RobotContainer robotContainer;
    double tiltAngle = 0;
    double elevatorDistance = 0;
    

    public PositionCommand(RobotContainer robotContainer, OperatorButtons type) {
        this.type = type;
        this.robotContainer = robotContainer;

        addRequirements(robotContainer.grabberSubsystem);
        addRequirements(robotContainer.elevatorSubsystem);

        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        started = false;
        logf("Command Started for %s\n", type);
        timeOut = 200;
        switch (type) {
            case HOME:
                tiltAngle = 50;
                elevatorDistance = 0;
                break;
            case CHUTE: // TODO
                tiltAngle = 90;
                elevatorDistance = 40;
                break;
            case SHELF: //  TODO
                tiltAngle = 90;
                elevatorDistance = 15;
                break;
            case GROUND:
                tiltAngle = 130;
                elevatorDistance = 10;
                break;
            case HIGH:
                tiltAngle = 70;
                elevatorDistance = 105;
                break;
            case MIDDLE:
                tiltAngle = 70;
                elevatorDistance = 60;
                break;
            case LOW:
                tiltAngle = 70;
                elevatorDistance = 0;
                break;
            case CONE:
                return;
            case CUBE:
                return;
        }

        robotContainer.grabberSubsystem.setTiltAngle(tiltAngle);
        robotContainer.elevatorSubsystem.setElevatorPos(elevatorDistance);
        logf("Init Position Command tilt angle:%.2f elevator distance:%.2f\n", tiltAngle, elevatorDistance);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }
    boolean started = false;

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // Fix case when grabber hits bumper upon startup
        if (robotContainer.grabberSubsystem.isElevatorSafeToMove() && !started) {
            robotContainer.elevatorSubsystem.setElevatorPos(elevatorDistance);
            started = true;
        }

        if (robotContainer.grabberSubsystem.atSetPoint() && robotContainer.elevatorSubsystem.atSetPoint()) {
            logf("Requested Positon Reached for type:%s\n", type);
            return true;
        }
        timeOut--;
        if (timeOut < 0) {
            logf("Timeout Position Command for %s\n", type);
            return true;
        }
        return false;
    }
}