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
    private boolean powerMode = true;
    private boolean elevatorPowered = false;

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
        if (powerMode) {
            double pos = elevatorSubsystem.getLastElevatorPositionInches();
            // Control Elevator in power mode
            if (pov == 270) {  // Up 
                //elevatorSubsystem.setPower(-.2, false);
                elevatorSubsystem.setElevatorPos(pos - .5);
                elevatorPowered = true;
            }
            if (pov == 90) {  // Down
                elevatorSubsystem.setElevatorPos(pos + .5);
                //elevatorSubsystem.setPower(.2, false);
                elevatorPowered = true;
            }
            if (!(pov == 270 || pov == 90) && elevatorPowered) { //Stop
                //elevatorSubsystem.setPower(0, false);
                elevatorPowered = false;
            }
        }
        else if (pov != lastPov) {
            double pos = elevatorSubsystem.getLastElevatorPositionInches();
            // Control Elevator in position mode
            if (pov == 270) {
                elevatorSubsystem.setElevatorPos(pos + 10);
            }
            if (pov == 90) {
                elevatorSubsystem.setElevatorPos(pos - 10);
            }
            lastPov = pov;
        }
    }
}
