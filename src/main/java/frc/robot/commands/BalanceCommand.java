package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.DrivetrainSubsystem;

public class BalanceCommand extends CommandBase {
    public final static double PITCH_THRESHOLD = 1;
    public final static double ROLL_THRESHOLD = 1;
    public final static double MAX_VELOCITY = 0.004;
    private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
    private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
    
    private final ProfiledPIDController xController = new ProfiledPIDController(0.0035, 0, 0, X_CONSTRAINTS);
    private final ProfiledPIDController yController = new ProfiledPIDController(0.0035, 0, 0, Y_CONSTRAINTS);
    

    DrivetrainSubsystem drivetrainSubsystem;

    double zeroPitch = 0;
    double zeroRoll = 0;

    public BalanceCommand(DrivetrainSubsystem drivetrainSubsystem) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        addRequirements(drivetrainSubsystem);
        xController.setTolerance(2);
        yController.setTolerance(2);
    }

    public void zeroGyroscope() {
        zeroRoll = drivetrainSubsystem.m_navx.getRoll();
        zeroPitch = drivetrainSubsystem.m_navx.getPitch();
        xController.reset(drivetrainSubsystem.m_navx.getRoll());
        yController.reset(drivetrainSubsystem.m_navx.getPitch());
    }

    @Override
    public void initialize() {
        xController.reset(drivetrainSubsystem.m_navx.getRoll());
        yController.reset(drivetrainSubsystem.m_navx.getPitch());
        xController.setGoal(0);
        yController.setGoal(0);
    }

    @Override
    public void execute() {
        double pitch = drivetrainSubsystem.m_navx.getPitch() - zeroPitch;
        double roll = drivetrainSubsystem.m_navx.getRoll() - zeroRoll;
        if (Robot.count % 20 == 0) {
            SmartDashboard.putNumber("Pitch", pitch);
            SmartDashboard.putNumber("Roll", roll);
        }
        

        if (Math.abs(roll) < ROLL_THRESHOLD &&
                Math.abs(pitch) < PITCH_THRESHOLD) {
            drivetrainSubsystem.stop();
            return;
        }
        
        double xSpeed = -xController.calculate(pitch); // correct direction
        double ySpeed = yController.calculate(roll);
        double omegaSpeed = 0;
        
        if (Math.abs(roll) < ROLL_THRESHOLD) {
            ySpeed=0;
        }
        if (Math.abs(pitch) < PITCH_THRESHOLD) {
            xSpeed=0;
        }
        if (Robot.count % 20 == 0) {
            SmartDashboard.putNumber("xSpeed", xSpeed);
            SmartDashboard.putNumber("ySpeed", ySpeed);
        }

        drivetrainSubsystem.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, omegaSpeed, new Rotation2d(0)));
    }

    double getSpeed(double angleInDegrees) {
        return angleInDegrees * MAX_VELOCITY;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrainSubsystem.stop();
    }

    
}