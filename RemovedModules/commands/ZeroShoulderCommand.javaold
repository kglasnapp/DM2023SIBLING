package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ZeroShoulderCommand extends CommandBase {
    ArmSubsystem armSubsystem;
    

    public ZeroShoulderCommand(ArmSubsystem armSubsystem) {
        this.armSubsystem = armSubsystem;       
        addRequirements(armSubsystem);
    }

    @Override
    public void execute() {
        armSubsystem.setShoulderSpeed(-1);
    }
    
    @Override
    public boolean isFinished() {
        return armSubsystem.getReverseLimitSwitch(armSubsystem.shoulderMotor);
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.setExtenderSpeed(0);
    }
}
