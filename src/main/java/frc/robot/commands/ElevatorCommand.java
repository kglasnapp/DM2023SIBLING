package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorCommand extends CommandBase {
    static ElevatorSubsystem elevatorSubsystem;
    CommandXboxController operatorController;
    int lastPov = -1;

    enum STATE {
        IDLE, HOMEING, RAISED, DROPED
    }

    STATE state = STATE.IDLE;

    public ElevatorCommand(ElevatorSubsystem elevatorSubsystem, CommandXboxController operatorController) {
        ElevatorCommand.elevatorSubsystem = elevatorSubsystem;
        this.operatorController = operatorController;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
        elevatorSubsystem.setElevatorPos(0);
    }

    @Override
    public void execute() {
        boolean left = operatorController.getHID().getRawButtonPressed(5);
        boolean right = operatorController.getHID().getRawButtonPressed(6);
        double pos = elevatorSubsystem.getLastElevatorPositionInches();

        if (left) {
            elevatorSubsystem.setElevatorPos(pos + 3);
        }

        if (right) {
            elevatorSubsystem.setElevatorPos(pos - 3);
        }
    }
}
