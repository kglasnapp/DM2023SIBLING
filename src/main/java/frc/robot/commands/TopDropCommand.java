package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GrabberTiltSubsystem;

public class TopDropCommand extends CommandBase {

    double ELEVATOR_THRESHOLD = 10;
    double TILT_THRESHOLD = 10;

    double CONE_ANGLE = 10;
    double CUBE_ANGLE = 10;

    ElevatorSubsystem elevatorSubsystem;
    GrabberTiltSubsystem grabberSubsystem;
    double targetAngle = 0;

    public TopDropCommand(ElevatorSubsystem elevatorSubsystem, GrabberTiltSubsystem grabberSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.grabberSubsystem = grabberSubsystem;
                
        if (RobotContainer.robotMode == RobotContainer.RobotMode.Cone) {
            targetAngle = CONE_ANGLE;
        } else {
            targetAngle = CUBE_ANGLE;
        }

        addRequirements(elevatorSubsystem, grabberSubsystem);
    }

    @Override
    public void initialize() {
        elevatorSubsystem.setElevatorPos(ElevatorSubsystem.HIGH_POS);
        grabberSubsystem.setTiltAngle(targetAngle);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(elevatorSubsystem.getElevatorPos() - ElevatorSubsystem.HIGH_POS) < ELEVATOR_THRESHOLD
            && Math.abs(grabberSubsystem.getTiltPos() - targetAngle) < TILT_THRESHOLD;
    }
}
