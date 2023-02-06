package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.Util;
import frc.robot.subsystems.DrivetrainSubsystem;
import static frc.robot.Util.logf;

public class StraightPathCommand extends CommandBase {
    DrivetrainSubsystem drivetrainSubsystem;

    private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
    private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
    private static final TrapezoidProfile.Constraints OMEGA_CONSTRATINTS = new TrapezoidProfile.Constraints(
            Math.toRadians(180), Math.toRadians(180));
    private final ProfiledPIDController xController = new ProfiledPIDController(1, 0, 0, X_CONSTRAINTS);
    private final ProfiledPIDController yController = new ProfiledPIDController(1, 0, 0, Y_CONSTRAINTS);
    private final ProfiledPIDController omegaController = new ProfiledPIDController(.5, 0, 0, OMEGA_CONSTRATINTS);
    private final Supplier<Pose2d> poseProvider;
    Pose2d initialPose;
    Supplier<Pose2d> destination;

    public StraightPathCommand(DrivetrainSubsystem drivetrainSubsystem, Supplier<Pose2d> poseProvider,
            Supplier<Pose2d> destination) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.destination = destination;
        this.poseProvider = poseProvider;

        xController.setTolerance(0.005);
        yController.setTolerance(0.005);
        omegaController.setTolerance(Units.degreesToRadians(3));
        omegaController.enableContinuousInput(-Math.PI, Math.PI);
        addRequirements(drivetrainSubsystem);
    }

    double initialTime = RobotController.getFPGATime();

    public void initialize() {

        initialTime = RobotController.getFPGATime();
        initialPose = poseProvider.get();
        omegaController.reset(initialPose.getRotation().getRadians());
        xController.reset(initialPose.getX());
        yController.reset(initialPose.getY());
        logf("Init Path Follow pose:<%.2f,%.2f,%.2f>\n", initialPose.getX(), initialPose.getY(),
                initialPose.getRotation().getDegrees());
    }

    double getIntermediateGoal(double endPosition, double initialPosition, double targetTime, double currentTime) {
        double velocity = (endPosition - initialPosition) / targetTime;
        if (targetTime < currentTime) {
            return endPosition;
        }
        return initialPosition + velocity * currentTime;
    }

    @Override
    public void execute() {
        logf("executing path follow command\n");
        double currentTime = RobotController.getFPGATime() - initialTime;

        double goalX = getIntermediateGoal(destination.get().getX(), initialPose.getX(), 3, currentTime);
        double goalY = getIntermediateGoal(destination.get().getY(), initialPose.getY(), 3, currentTime);
        double goalAngle = getIntermediateGoal(destination.get().getRotation().getDegrees(),
                initialPose.getRotation().getDegrees(), 3, currentTime);
        xController.setGoal(goalX);
        yController.setGoal(goalY);
        omegaController.setGoal(Math.toRadians(goalAngle));
        var robotPose = poseProvider.get();

        if (Robot.count % 10 == 8) {
            SmartDashboard.putNumber("goal X", goalX);
            SmartDashboard.putNumber("goal Y", goalY);
            SmartDashboard.putNumber("goal A", goalAngle);
        }

        if (Robot.count % 20 == 3) {
            // logf("Path time:%.3f goal:<%.2f,%.2f,%.2f> robot pose:<%.2f,%.2f,%.2f>\n",
            // (currentTime - initialTime) / 1000000,
            // goal.poseMeters.getX(), goal.poseMeters.getX(),
            // goal.poseMeters.getRotation().getDegrees(),
            // robotPose.getX(), robotPose.getY(), robotPose.getRotation().getDegrees());
        }
        var xSpeed = xController.calculate(robotPose.getX());
        if (xController.atGoal()) {
            xSpeed = 0;
        }

        var ySpeed = yController.calculate(robotPose.getY());
        if (yController.atGoal()) {
            ySpeed = 0;
        }

        var omegaSpeed = omegaController.calculate(robotPose.getRotation().getRadians());
        if (omegaController.atGoal()) {
            omegaSpeed = 0;
        }

        drivetrainSubsystem.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, omegaSpeed, robotPose.getRotation()));
    }

    @Override
    public boolean isFinished() {
        double currentTime = RobotController.getFPGATime();
        var robotPose = poseProvider.get();
        boolean atGoalX = Math.abs(robotPose.getX() - destination.get().getX()) < 0.01;
        boolean atGoalY = Math.abs(robotPose.getY() - destination.get().getY()) < 0.01;
        boolean atGoalO = Math.abs((Util.normalizeAngle(robotPose.getRotation().getDegrees() -
                destination.get().getRotation().getDegrees()))) < 2;
        logf("Path Follow Complete time:%3f robot pose:<%.2f,%.2f,%.2f, %b, %b, %b>\n",
                (currentTime - initialTime) / 1000000,
                robotPose.getX(), robotPose.getY(), robotPose.getRotation().getDegrees(), atGoalX, atGoalY, atGoalO);
        return atGoalX &&
                atGoalY &&
                atGoalO;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrainSubsystem.stop();
    }
}
