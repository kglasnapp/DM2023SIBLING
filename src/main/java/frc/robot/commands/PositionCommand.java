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
        robotContainer.grabberSubsystem.setTiltAngle(5);
        robotContainer.elevatorSubsystem.setElevatorPos(1);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        logf("Position Command for %s\n", type);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        timeOut--;
        if (timeOut < 0) {
            return true;
        }
        return false;
    }
}