package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import static frc.robot.utilities.Util.logf;
//import static frc.robot.Constants.isMini;

public class DriveToObjectCommand extends CommandBase {
    private DrivetrainSubsystem drivetrainSubsystem;
    private CoralSubsystem coral;
    private double x;
    private double area;
    private double finishArea = 200000;
    private double finishX = 0.002;
    private String type;

    /** Creates a new ReplaceMeCommand. */
    public DriveToObjectCommand(DrivetrainSubsystem drivetrainSubsystem, String type) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.drivetrainSubsystem = drivetrainSubsystem;
        addRequirements(drivetrainSubsystem);
        this.type = type;
        coral = RobotContainer.coralSubsystem;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double omegaSpeed = 0;
        double xSpeed = 0;
        x = -coral.x;
        if (Math.abs(x) > finishX) {
            omegaSpeed = x * x * 5;
            if (x < 0) {
                omegaSpeed = -omegaSpeed;
            }

        }
        area = coral.area;
        if (area < finishArea) {
            xSpeed = 0.01;
        }
        if (Robot.count % 5 == 0) {
            logf("Coral type:%s x:%.4f y:%.4f area:%.0f PerCent:%.0f xSpeed:%.5f omegaSpeed:%.5f\n",
                    coral.type, x, coral.y, area, coral.percent, xSpeed, omegaSpeed);

        }

        drivetrainSubsystem.drive(new ChassisSpeeds(xSpeed, 0, Math.toRadians(omegaSpeed)));
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (area > finishArea) {
            logf("Drive to Object area:%.0f percent:%.1f Requesed Type:%s\n", area, coral.percent, type);
            return true;
        }
        return false;
        // if (!coral.type.equals(type)) {
        //     logf("Coral invalid type: coral:%s requested:%s\n", coral.type, type);
        //     return true;
        // }
        // return (Math.abs(x) < finishX && coral.area > finishArea);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        drivetrainSubsystem.drive(new ChassisSpeeds(0, 0, 0));
        logf("Drive to object end area:%.0f \n", coral.area);
    }
}