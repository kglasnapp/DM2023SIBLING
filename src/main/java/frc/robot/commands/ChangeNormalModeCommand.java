package frc.robot.commands;

import com.swervedrivespecialties.swervelib.SwerveModuleFactory;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ChangeNormalModeCommand extends CommandBase{
    @Override
          public void initialize() {
            SwerveModuleFactory.powerRatio = SwerveModuleFactory.NORMAL;
          }
    
          @Override
          public boolean isFinished() {
            return true;
          }
}
