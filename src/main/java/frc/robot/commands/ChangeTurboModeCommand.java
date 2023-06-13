package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utilities.SwerveModule;

public class ChangeTurboModeCommand extends CommandBase{
    @Override
          public void initialize() {
            SwerveModule.powerRatio = SwerveModule.TURBO;
          }
    
          @Override
          public boolean isFinished() {
            return true;
          }
}
