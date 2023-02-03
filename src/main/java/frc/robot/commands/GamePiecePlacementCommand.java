package frc.robot.commands;

import static frc.robot.utilities.Util.logf;

import org.opencv.core.Point;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;

public class GamePiecePlacementCommand extends CommandBase {
    Pose2d driveTrainPoseTarget;
    Point armTarget;
    DrivetrainSubsystem drivetrainSubsystem;
    ArmSubsystem armSubsystem;
    PoseEstimatorSubsystem poseEstimatorSubsystem;
    PathFollowCommand pathFollowCommand;
    ShoulderCommand shoulderCommand;
    ExtenderCommand extenderCommand;
    StraightPathCommand straightPathCommand;
    // Trajectory trajectory;
    
    Command compositeCommand;

    public final static Pose2d driveTrainPoseTargets[] = new Pose2d[] {
        // new Pose2d(new Translation2d(14.55, 4.91), new Rotation2d(0,0)),
        new Pose2d(new Translation2d(1.79, 2.88), new Rotation2d(Math.toRadians(180))), // this worked!
       // new Pose2d(new Translation2d(16.61, 4.38), new Rotation2d(Math.toRadians(180))),
        
        new Pose2d(new Translation2d(0.0, 0.0), new Rotation2d(0,0)),
        new Pose2d(new Translation2d(0.0, 0.0), new Rotation2d(0,0)),
        new Pose2d(new Translation2d(0.0, 0.0), new Rotation2d(0,0)),
        
        new Pose2d(new Translation2d(0.0, 0.0), new Rotation2d(0,0)),
        new Pose2d(new Translation2d(0.0, 0.0), new Rotation2d(0,0)),
        new Pose2d(new Translation2d(0.0, 0.0), new Rotation2d(0,0))
    };

    public final static Point armTargets[] = new Point[] {
        new Point(1.05,-56), // floor
        new Point(0,0), // middle
        new Point(0,0)  // highest
    };

    public GamePiecePlacementCommand(
            DrivetrainSubsystem drivetrainSubsystem,
            ArmSubsystem armSubsystem,
            PoseEstimatorSubsystem poseEstimatorSubsystem,
            Pose2d driveTrainPoseTarget, 
            Point armTarget) {
        this.driveTrainPoseTarget = driveTrainPoseTarget;
        this.armTarget = armTarget;
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.armSubsystem = armSubsystem;
        this.poseEstimatorSubsystem = poseEstimatorSubsystem;
        addRequirements(drivetrainSubsystem);
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        // Create config for trajectory
        // TrajectoryConfig config = new TrajectoryConfig(
        //         1.0, // kMaxSpeedMetersPerSecond,
        //         1.0) // AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        //              // Add kinematics to ensure max speed is actually obeyed
        //                 .setKinematics(DrivetrainSubsystem.m_kinematics);
        // An example trajectory to follow. All units in meters.
       
        
        // trajectory = TrajectoryGenerator.generateTrajectory(
        //         // Start at the origin facing the +X direction
        //         poseEstimatorSubsystem.getCurrentPose(),
        //         // Pass through these two interior waypoints, making an 's' curve path
        //         new LinkedList<>(),
        //         // End 3 meters straight ahead of where we started, facing forward
        //         driveTrainPoseTarget,
        //         // Pass config
        //         config);
        // pathFollowCommand = new PathFollowCommand(trajectory, drivetrainSubsystem,
        //         poseEstimatorSubsystem::getCurrentPose);
        straightPathCommand =
            new StraightPathCommand(drivetrainSubsystem, poseEstimatorSubsystem::getCurrentPose,driveTrainPoseTarget);
        shoulderCommand = new ShoulderCommand(armSubsystem, armTarget.x);
        extenderCommand = new ExtenderCommand(armSubsystem, armTarget.y);
        
        compositeCommand = straightPathCommand.andThen(shoulderCommand).andThen(extenderCommand);
    }

    @Override
    public void execute() {
        logf("executing game piece placement command\n");
        // pathFollowCommand.schedule();
        compositeCommand.schedule();
    }

    @Override
    public void end(boolean interrupted) {  
        System.out.println("game piece commad end");
        straightPathCommand.end(interrupted);
        shoulderCommand.end(interrupted);
        extenderCommand.end(interrupted);
        compositeCommand.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return compositeCommand.isFinished();
        // return pathFollowCommand.isFinished();
    }
}
