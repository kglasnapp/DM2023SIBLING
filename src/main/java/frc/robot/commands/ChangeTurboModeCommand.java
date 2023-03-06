package frc.robot.commands;

import com.swervedrivespecialties.swervelib.SwerveModuleFactory;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ChangeTurboModeCommand extends CommandBase{
    @Override
          public void initialize() {
            SwerveModuleFactory.powerRatio = SwerveModuleFactory.TURBO;
          }
    
          @Override
          public boolean isFinished() {
            return true;
          }
}
