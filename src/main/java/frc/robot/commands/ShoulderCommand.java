package frc.robot.commands;

import static frc.robot.utilities.Util.logf;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.ArmSubsystem;

public class ShoulderCommand extends CommandBase {
    /**
     * TOLERACE is the error that we are ok with at the end of the command (in
     * incshes)
     */
    public final static double TOLERANCE = 0.02;
    private TrapezoidProfile shoulderTrapezoidProfile;
    ArmSubsystem armSubsystem; 
    double shoulderInitial;
    double shoulderGoal;
    double initialTime = 0;
    /**
     * We create a command with a position goal in inches.
     * f\
     * @param shoulderGoal
     */
    public ShoulderCommand(ArmSubsystem armSubsystem, double shoulderGoal) {
        this.armSubsystem = armSubsystem;
        this.shoulderGoal = shoulderGoal;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        this.shoulderInitial = armSubsystem.getShoulderRevs();
        TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(4, 2);
        TrapezoidProfile.State shoulderGoalState = new TrapezoidProfile.State(shoulderGoal - shoulderInitial, 0);
        shoulderTrapezoidProfile = new TrapezoidProfile(constraints, shoulderGoalState);
        initialTime = RobotController.getFPGATime();
    }

    @Override
    public void execute() {
        double currentTime = RobotController.getFPGATime();
        double elapsedSec = (currentTime - initialTime) / 1000000;
        TrapezoidProfile.State intermediateShoulderState = shoulderTrapezoidProfile.calculate(elapsedSec);
        double intermediateShoulderGoal = intermediateShoulderState.position;
        armSubsystem.setMotorToPosition(armSubsystem.shoulderMotor, (intermediateShoulderGoal + shoulderInitial) * 2000 * 48);

        if (Robot.count % 15 == 5) {
            SmartDashboard.putNumber("Shld Time", elapsedSec);
            double revolutions = armSubsystem.getShoulderRevs();
            SmartDashboard.putNumber("Shld Revs", revolutions);
            SmartDashboard.putNumber("Shld Int Goal", intermediateShoulderGoal);
            SmartDashboard.putNumber("Shld Fin Goal", shoulderGoal);
            logf("Time:%.1f Pos:%.2f intGoal+initial:%.2f goal:%.2f initial:%.2f\n", elapsedSec, revolutions,
                    intermediateShoulderGoal + shoulderInitial, shoulderGoal, shoulderInitial);
        }
    }

    @Override
    public boolean isFinished() {
        return (Math.abs(armSubsystem.getShoulderRevs() - shoulderGoal) < TOLERANCE);
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.setShoulderSpeed(0);        
        SmartDashboard.putNumber("ShlSpd", 0);
    }

    double getShoulderAngle(double x, double y) {
        return -Math.acos((x * x + y * y - ArmSubsystem.SHOULDER_TO_ELBOW * ArmSubsystem.SHOULDER_TO_ELBOW
                - ArmSubsystem.ELBOW_TO_WRIST * ArmSubsystem.ELBOW_TO_WRIST)
                / (2 * ArmSubsystem.ELBOW_TO_WRIST * ArmSubsystem.SHOULDER_TO_ELBOW));
    }
 
    double getElbowAngle(double x, double y, double shoulderAngle) {
        return Math.atan2(y, x) + Math.atan2(ArmSubsystem.ELBOW_TO_WRIST * Math.sin(shoulderAngle),
                ArmSubsystem.SHOULDER_TO_ELBOW + ArmSubsystem.ELBOW_TO_WRIST * Math.cos(shoulderAngle));
    }

}
