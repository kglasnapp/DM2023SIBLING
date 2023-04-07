package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.utilities.Util;

public class RotateCommand extends CommandBase {
    double initialTime = 0;
    double targetRotationAngle = 180;
    double goal = 0;
    double initialAngle = 0;
    DrivetrainSubsystem drivetrainSubsystem;

    public RotateCommand(DrivetrainSubsystem drivetrainSubsystem) {
        this(drivetrainSubsystem, 180);
    }

    public RotateCommand(DrivetrainSubsystem drivetrainSubsystem, double targetRotationAngleDegrees) {
        this.drivetrainSubsystem = drivetrainSubsystem;        
        this.targetRotationAngle = normalizeAngle(targetRotationAngleDegrees);
    }

    @Override
    public void initialize() {
        addRequirements(drivetrainSubsystem);
        initialAngle = normalizeAngle(drivetrainSubsystem.getGyroscopeRotation().getDegrees());
        goal = normalizeAngle(initialAngle + targetRotationAngle);                
    }   

    @Override
    public void execute() {
        double omegaSpeed = ((goal - initialAngle) - (normalizeAngle(drivetrainSubsystem.getGyroscopeRotation().getDegrees())-initialAngle))/25;
        drivetrainSubsystem.drive(new ChassisSpeeds(0,0, Math.toRadians(omegaSpeed)));
    }

    double normalizeAngle(double angle) {
        return Util.unNormalilzeAngle(angle);
    }

    @Override
    public boolean isFinished() {
        double currentAngle = normalizeAngle(drivetrainSubsystem.getGyroscopeRotation().getDegrees());
        return  Math.abs(goal - currentAngle) < 1;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrainSubsystem.stop();
    }
}
