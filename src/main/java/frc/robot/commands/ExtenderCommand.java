package frc.robot.commands;

import static frc.robot.utilities.Util.logf;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.ArmSubsystem;

public class ExtenderCommand extends CommandBase {
    /**
     * TOLERACE is the error that we are ok with at the end of the command (in
     * inches)
     */
    public final static double TOLERANCE = 5000;
    ArmSubsystem armSubsystem;
    double extenderGoal;

    double initialTime = 0;

    private TrapezoidProfile extenderTrapezoidProfile;
    double extenderInitial;
    
    /**
     * We create a command with a position goal in inches.
     * 
     * @param extenderGoal
     */
    public ExtenderCommand(ArmSubsystem armSubsystem, double goal) {
        this.armSubsystem = armSubsystem;
        this.extenderGoal = goal;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(580000, 400000);
        this.extenderInitial = armSubsystem.getExtenderPos();
        TrapezoidProfile.State extenderGoalState = new TrapezoidProfile.State(extenderGoal - extenderInitial, 0);
        extenderTrapezoidProfile = new TrapezoidProfile(constraints, extenderGoalState);
        initialTime = RobotController.getFPGATime();
    }

    @Override
    public void execute() {
        double currentTime = RobotController.getFPGATime();
        double elapsedSec = (currentTime - initialTime) / 1000000;

        TrapezoidProfile.State intermediateExtenderState = extenderTrapezoidProfile.calculate(elapsedSec);
        double intermediateExtenderGoal = intermediateExtenderState.position;

        armSubsystem.setMotorToPosition(armSubsystem.extenderMotor, (intermediateExtenderGoal + extenderInitial));
        if (Robot.count % 15 == 10) {
            double position = armSubsystem.getExtenderPos();
            // SmartDashboard.putNumber("Ext Goal", extenderGoal);
            logf("Time:%.1f Extender Pos:%.2f intGoal+initial:%.2f goal:%.2f initial:%.2f\n", elapsedSec, position,
                    intermediateExtenderGoal + extenderInitial, extenderGoal, extenderInitial);
        }
    }

    @Override
    public boolean isFinished() {
        return (Math.abs(armSubsystem.getExtenderPos() - extenderGoal) < TOLERANCE);
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.setExtenderSpeed(0);
        // SmartDashboard.putNumber("ExtSpd", 0);
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
