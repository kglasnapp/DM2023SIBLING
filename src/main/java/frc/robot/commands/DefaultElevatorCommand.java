package frc.robot.commands;

import static frc.robot.Util.logf;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GrabberTiltSubsystem;
import frc.robot.RobotContainer;

public class DefaultElevatorCommand extends CommandBase {
    ElevatorSubsystem elevatorSubsystem;
    GrabberTiltSubsystem grabberSubsystem;
    CommandXboxController operatorController;
    int lastPov = -1;
    private boolean powerMode = false;

    public DefaultElevatorCommand(ElevatorSubsystem elevatorSubsystem, GrabberTiltSubsystem grabberSubsystem,
            CommandXboxController operatorController) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.grabberSubsystem = grabberSubsystem;
        this.operatorController = operatorController;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
        logf("Init Elevator Default Command\n");
    }

    @Override
    public void execute() {
        int pov = RobotContainer.getDriverPov();
        if (pov != lastPov) {
            double pos = elevatorSubsystem.getLastElevatorPositionInches();
            if (powerMode) {
                // Control Elevator in power mode
                if (pov == 270) {  // Up 
                    elevatorSubsystem.setPower(-.2, false);
                }
                if (pov == 90) {  // Down
                    elevatorSubsystem.setPower(.2, false);
                }
                if (!(pov == 270 || pov == 90)) {
                    elevatorSubsystem.setPower(0, false);
                }
            } else {
                // Control Elevator in position mode
                if (pov == 270) {
                    elevatorSubsystem.setElevatorPos(pos + 10);
                }
                if (pov == 90) {
                    elevatorSubsystem.setElevatorPos(pos - 10);
                }
            }
            lastPov = pov;
        }
    }
}
