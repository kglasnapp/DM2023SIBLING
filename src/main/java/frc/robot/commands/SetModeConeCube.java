package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.RobotMode;

public class SetModeConeCube extends CommandBase {
    RobotMode mode;

    public SetModeConeCube(RobotMode mode) {
        this.mode = mode;
    }

    @Override
    public void initialize() {
        RobotContainer.instance.setMode(mode);
    }

    @Override
    public boolean isFinished() {
        //logf("Set Mode:%S\n", mode);
        return true;
    }
}
