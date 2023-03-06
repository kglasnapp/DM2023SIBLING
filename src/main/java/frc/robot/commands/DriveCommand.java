package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveCommand extends CommandBase {

    DrivetrainSubsystem drivetrainSubsystem;
    double xSpeed;
    double ySpeed;
    double angleSpeed;

    boolean isFinished = false;

    public DriveCommand(DrivetrainSubsystem drivetrainSubsystem, double xSpeed, double ySpeed, double angleSpeed) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;
        this.angleSpeed = angleSpeed;
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        drivetrainSubsystem.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        xSpeed,
                        ySpeed,
                        angleSpeed,
                        drivetrainSubsystem.getGyroscopeRotation()));
        isFinished = true;
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

    @Override
    public void end(boolean interrupted) {
        
    }
    
}
