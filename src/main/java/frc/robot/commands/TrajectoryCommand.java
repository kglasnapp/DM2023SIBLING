package frc.robot.commands;

import java.util.ArrayList;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.Util;
import frc.robot.subsystems.DrivetrainSubsystem;
import static frc.robot.Util.logf;

public class TrajectoryCommand extends CommandBase {
    DrivetrainSubsystem drivetrainSubsystem;

    private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(7, 1.5);
    private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(7, 1.5);
    private static final TrapezoidProfile.Constraints OMEGA_CONSTRATINTS = new TrapezoidProfile.Constraints(
            Math.toRadians(180), Math.toRadians(180));
    private final ProfiledPIDController xController = new ProfiledPIDController(0.1, 0, 0, X_CONSTRAINTS);
    private final ProfiledPIDController yController = new ProfiledPIDController(0.1, 0, 0, Y_CONSTRAINTS);
    private final ProfiledPIDController omegaController = new ProfiledPIDController(0.015, 0, 0, OMEGA_CONSTRATINTS);
    private final Supplier<Pose2d> poseProvider;
    Pose2d initialPose;
    Supplier<Pose2d> destinationProvider;
    Trajectory trajectory;
    long initialTime;

    public TrajectoryCommand(DrivetrainSubsystem drivetrainSubsystem, Supplier<Pose2d> poseProvider) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.poseProvider = poseProvider;
        init();
    }

    public static Trajectory generateTrajectory(Pose2d startPose) {

        // 2018 cross scale auto waypoints.

        var endPose = new Pose2d(4.49, 5.08,
                Rotation2d.fromDegrees(0));

        var interiorWaypoints = new ArrayList<Translation2d>();
        interiorWaypoints.add(new Translation2d(2.8, 5.08));
        // interiorWaypoints.add(new Translation2d(Units.feetToMeters(21.04), Units.feetToMeters(18.23)));

        TrajectoryConfig config = new TrajectoryConfig(1, 0.5);

        return TrajectoryGenerator.generateTrajectory(
                startPose,
                interiorWaypoints,
                endPose,
                config);
    }

    void init() {
        xController.setTolerance(0.005);
        yController.setTolerance(0.005);
        omegaController.setTolerance(Units.degreesToRadians(1));
        omegaController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(drivetrainSubsystem);
    }

    public void initialize() {
        initialPose = poseProvider.get();
        logf("Init Path Follow pose:<%.2f,%.2f,%.2f> yaw:%.2f\n", initialPose.getX(), initialPose.getY(),
                initialPose.getRotation().getDegrees(), drivetrainSubsystem.m_navx.getYaw());
        initialTime = RobotController.getFPGATime();
        trajectory = generateTrajectory(initialPose);
        omegaController.reset(initialPose.getRotation().getRadians());
        xController.reset(initialPose.getX());
        yController.reset(initialPose.getY());

    }

    @Override
    public void execute() {

        var robotPose = poseProvider.get();

        // if (Robot.count % 10 == 8) {
        //     SmartDashboard.putNumber("goal X", goalX);
        //     SmartDashboard.putNumber("goal Y", goalY);
        //     SmartDashboard.putNumber("goal A", goalAngle);
        // }
        Trajectory.State goal = trajectory
                .sample(2 * ((double) (RobotController.getFPGATime() - initialTime)) / 1000000.0); // sample the trajectory at 3.4 seconds from the beginning
        if (Robot.count % 10 == 3) {
            logf("Path time:%.3f goal:<%.2f,%.2f,%.2f> robot pose:<%.2f,%.2f,%.2f>\n",
                    (RobotController.getFPGATime() - initialTime) / 1000000.0,
                    goal.poseMeters.getX(), goal.poseMeters.getY(),
                    goal.poseMeters.getRotation().getDegrees(),
                    robotPose.getX(), robotPose.getY(), robotPose.getRotation().getDegrees());
        }

        xController.setGoal(goal.poseMeters.getX());
        yController.setGoal(goal.poseMeters.getY());
        omegaController.setGoal(goal.poseMeters.getRotation().getRadians());
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
        Pose2d destination = trajectory.getStates().get(trajectory.getStates().size() - 1).poseMeters;
        var robotPose = poseProvider.get();
        boolean atGoalX = Math.abs(robotPose.getX() - destination.getX()) < 0.01;
        boolean atGoalY = Math.abs(robotPose.getY() - destination.getY()) < 0.01;
        boolean atGoalO = Math.abs((Util.normalizeAngle(robotPose.getRotation().getDegrees() -
                destination.getRotation().getDegrees()))) < 1;
        boolean finished = atGoalX && atGoalY && atGoalO;
        if (finished) {
            logf("Path Follow Complete time:%3f robot pose:<%.2f,%.2f,%.2f, %b, %b, %b> yaw:%.2f\n",
                    (RobotController.getFPGATime() - initialTime) / 1000000.0,
                    robotPose.getX(), robotPose.getY(), robotPose.getRotation().getDegrees(), atGoalX, atGoalY,
                    atGoalO, drivetrainSubsystem.m_navx.getYaw());
        }
        return finished;

    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            logf("*********** Finished straight path command with  interrupted:%b\n",
                    interrupted);
        }
        drivetrainSubsystem.stop();
    }
}
