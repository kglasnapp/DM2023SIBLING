package frc.robot.utilities;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.BalanceCommand;
import frc.robot.commands.ExtenderCommand;
import frc.robot.commands.GrabberCommand;
import frc.robot.commands.RobotOrientedDriveCommand;
import frc.robot.commands.ShoulderCommand;
import frc.robot.commands.ZeroExtenderCommand;
import frc.robot.commands.ZeroShoulderCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.GrabberSubsystem;

public class AutonomousCommandFactory {

    public static Command getAutonomousSimpleAndLeftOutCommand(DrivetrainSubsystem m_drivetrainSubsystem,
                                                 ArmSubsystem m_armSubsystem, 
                                                 GrabberSubsystem grabberSubsystem) {
        return getAutonomousSimpleCommand(m_drivetrainSubsystem,m_armSubsystem, grabberSubsystem)
          .andThen(new RobotOrientedDriveCommand(m_drivetrainSubsystem, 0, -0.02, 0, 750.0))          
          .andThen(new RobotOrientedDriveCommand(m_drivetrainSubsystem, -0.02, 0, 0, 5500))          
          .andThen (new RobotOrientedDriveCommand(m_drivetrainSubsystem, 0, 0, 0, 500));
      }
    
      public static Command getAutonomousSimpleAndRightOutCommand(DrivetrainSubsystem m_drivetrainSubsystem,
                                                    ArmSubsystem m_armSubsystem, 
                                                    GrabberSubsystem grabberSubsystem) {
        return getAutonomousSimpleCommand(m_drivetrainSubsystem,m_armSubsystem, grabberSubsystem)          
          .andThen(new RobotOrientedDriveCommand(m_drivetrainSubsystem, 0, 0.02, 0, 750))
          .andThen(new RobotOrientedDriveCommand(m_drivetrainSubsystem, -0.02, 0, 0, 5500))        
          .andThen(new RobotOrientedDriveCommand(m_drivetrainSubsystem, 0, 0, 0, 500));
      }
    
      public static Command  getAutonomousSimpleCommand(DrivetrainSubsystem m_drivetrainSubsystem,
        ArmSubsystem m_armSubsystem, GrabberSubsystem grabberSubsystem) {        
        return new GrabberCommand(grabberSubsystem, false)
          .andThen(new ShoulderCommand(m_armSubsystem, 4000000))
          .andThen(new ExtenderCommand(m_armSubsystem, 400000000))
          .andThen(new WaitCommand(1))
          .andThen(new GrabberCommand(grabberSubsystem, true))
          .andThen(new WaitCommand(1))
          .andThen(new ZeroExtenderCommand(m_armSubsystem))
          .andThen(new ZeroShoulderCommand(m_armSubsystem))
          .andThen(new BalanceCommand(m_drivetrainSubsystem));
      } 
}
