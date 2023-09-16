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
        logf("Command Started for %s\n", type);
        timeOut = 200;
        switch (type) {
            case HOME:
                tiltAngle = 0;
                elevatorDistance = 0;
                break;
            case CHUTE:
                tiltAngle = 50;
                elevatorDistance = 40;
                break;
            case SHELF:
                tiltAngle = 20;
                elevatorDistance = 15;
                break;
            case GROUND:
                tiltAngle = 1;
                elevatorDistance = 1;
                break;
            case HIGH:
                tiltAngle = 8;
                elevatorDistance = 7;
                break;
            case MIDDLE:
                tiltAngle = 5;
                elevatorDistance = 5;
                break;
            case LOW:
                tiltAngle = 2;
                elevatorDistance = 3;
                break;
            case CONE:
                break;
            case CUBE:
                break;
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

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
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