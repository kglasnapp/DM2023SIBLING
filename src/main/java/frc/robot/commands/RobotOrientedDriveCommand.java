package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class RobotOrientedDriveCommand extends CommandBase {

    DrivetrainSubsystem drivetrainSubsystem;
    double xSpeed;
    double ySpeed;
    double angleSpeed;
    double duration;
    double initTime = 0;
    boolean isFinished = false;

    public RobotOrientedDriveCommand(DrivetrainSubsystem drivetrainSubsystem, double xSpeed, double ySpeed, double angleSpeed,
                                    double duration) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;
        this.angleSpeed = angleSpeed;
        this.duration = duration;
        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void initialize() {
        initTime = RobotController.getFPGATime()/1000;
        
    }

    @Override
    public void execute() {
        drivetrainSubsystem.drive(
            new ChassisSpeeds(
                        xSpeed,
                        ySpeed,
                        angleSpeed));                        
        isFinished = true;
    }

    @Override
    public boolean isFinished() {
        return RobotController.getFPGATime()/1000 > initTime + duration;
    }

    @Override
    public void end(boolean interrupted) {
        
    }
    
}
