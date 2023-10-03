package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Util;
import frc.robot.subsystems.DrivetrainSubsystem;
import static frc.robot.Util.logf;

public class StraightPathCommand extends CommandBase {
    DrivetrainSubsystem drivetrainSubsystem;

    private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(7, 1.5);
    private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(7, 1.5);
    private static final TrapezoidProfile.Constraints OMEGA_CONSTRATINTS = new TrapezoidProfile.Constraints(
            Math.toRadians(180), Math.toRadians(180));
    private final ProfiledPIDController xController = new ProfiledPIDController(0.15, 0, 0, X_CONSTRAINTS);
    private final ProfiledPIDController yController = new ProfiledPIDController(0.15, 0, 0, Y_CONSTRAINTS);
    private final ProfiledPIDController omegaController = new ProfiledPIDController(0.015, 0, 0, OMEGA_CONSTRATINTS);
    private final Supplier<Pose2d> poseProvider;
    Pose2d initialPose; 
    Supplier<Pose2d> destinationProvider;
    Pose2d destination;

    public StraightPathCommand(DrivetrainSubsystem drivetrainSubsystem, Supplier<Pose2d> poseProvider,
            Supplier<Pose2d> destination) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.destinationProvider = destination;
        this.poseProvider = poseProvider;
        init();
    }

    public StraightPathCommand(DrivetrainSubsystem drivetrainSubsystem, Supplier<Pose2d> poseProvider,
            Pose2d destination) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.destination = destination;
        this.poseProvider = poseProvider;
        init();
    }

    void init() {
        xController.setTolerance(0.005);
        yController.setTolerance(0.005);
        omegaController.setTolerance(Units.degreesToRadians(1));
        omegaController.enableContinuousInput(-Math.PI, Math.PI);
        addRequirements(drivetrainSubsystem);
    }

    double initialTime = RobotController.getFPGATime();

    public void initialize() {

        initialTime = RobotController.getFPGATime();
        if (poseProvider != null) {
            initialPose = poseProvider.get();
        } else {
            // TODO will this work
            initialPose = RobotContainer.instance.getLLPose();
        }
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
        // logf("executing path follow command\n");
        // double currentTime = RobotController.getFPGATime() - initialTime;
        // currentTime/= 1000;
        if (destinationProvider != null) {
            destination = destinationProvider.get();
        }

        // double goalX = getIntermediateGoal(destination.getX(), initialPose.getX(), 1000, currentTime);
        // double goalY = getIntermediateGoal(destination.getY(), initialPose.getY(), 1000, currentTime);
        // double goalAngle = getIntermediateGoal(destination.getRotation().getDegrees(),
        //         initialPose.getRotation().getDegrees(), 2, currentTime);
        xController.setGoal(destination.getX());
        yController.setGoal(destination.getY());
        omegaController.setGoal(destination.getRotation().getRadians());
        var robotPose = poseProvider.get();

        // if (Robot.count % 10 == 8) {
        //     SmartDashboard.putNumber("goal X", goalX);
        //     SmartDashboard.putNumber("goal Y", goalY);
        //     SmartDashboard.putNumber("goal A", goalAngle);
        // }

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
        //double currentTime = RobotController.getFPGATime();
        if (destinationProvider != null) {
            destination = destinationProvider.get();
        }
        var robotPose = poseProvider.get();
        boolean atGoalX = Math.abs(robotPose.getX() - destination.getX()) < 0.01;
        boolean atGoalY = Math.abs(robotPose.getY() - destination.getY()) < 0.01;
        boolean atGoalO = Math.abs((Util.normalizeAngle(robotPose.getRotation().getDegrees() -
                destination.getRotation().getDegrees()))) < 1;
        // logf("Path Follow Complete time:%3f robot pose:<%.2f,%.2f,%.2f, %b, %b, %b>\n",
        //         (currentTime - initialTime) / 1000000,
        //         robotPose.getX(), robotPose.getY(), robotPose.getRotation().getDegrees(), atGoalX, atGoalY, atGoalO);
        return atGoalX &&
                atGoalY &&
                atGoalO;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println(
                "****************************************** finished path command with destination: " + destination);
        drivetrainSubsystem.stop();
    }
}
