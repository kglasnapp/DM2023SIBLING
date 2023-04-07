package frc.robot.commands;

import com.swervedrivespecialties.swervelib.SwerveModuleFactory;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utilities.SwerveModule;

public class ChangeNormalModeCommand extends CommandBase{
    @Override
          public void initialize() {
            SwerveModule.powerRatio = SwerveModule.NORMAL;
          }
    
          @Override
          public boolean isFinished() {
            return true;
          }
}
