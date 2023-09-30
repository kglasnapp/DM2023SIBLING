package frc.robot.commands;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.OperatorButtons;
import frc.robot.RobotContainer.RobotMode;

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
        RobotMode mode = RobotContainer.robotMode;
        logf("Command Started for %s\n", type);
        timeOut = 200;
        switch (type) {
            case HOME:
                tiltAngle = 50;
                elevatorDistance = 0;
                break;
            case CHUTE: 
            if (mode == RobotMode.Cube) {
                tiltAngle = 70;
                elevatorDistance = 40;
            } else {
                tiltAngle = 130;
                elevatorDistance = 40;
            }
                break;
            case SHELF:
                tiltAngle = 90;
                elevatorDistance = 15;
                break;
            case GROUND:
                if (mode == RobotMode.Cube) {
                    tiltAngle = 132; 
                    elevatorDistance = 10;
                } else {
                    tiltAngle = 163;
                    elevatorDistance = 36;
                }
                break;
            case HIGH:
                if (mode == RobotMode.Cube) {
                    tiltAngle = 70; 
                    elevatorDistance = 60; // TODO was 110, resotre when elevator bearings fixed
                } else {
                    tiltAngle = 122;
                    elevatorDistance = 130;
                }
                break;
            case MIDDLE:
                if (mode == RobotMode.Cube) {
                    tiltAngle = 70;
                    elevatorDistance = 60;
                } else {
                    tiltAngle = 180;
                    elevatorDistance = 130;
                }
                break;
            case LOW:
                if (mode == RobotMode.Cube) {
                    tiltAngle = 90; 
                    elevatorDistance = 0; 
                } else {
                    tiltAngle = 180;
                    elevatorDistance = 50;
                }
                break;
            case CONE: // means nothing
                return;
            case CUBE: // means nothing
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
        if (!started && robotContainer.grabberSubsystem.isElevatorSafeToMove(elevatorDistance)) {
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