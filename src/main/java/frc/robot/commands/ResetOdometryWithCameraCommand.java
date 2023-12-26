package frc.robot.commands;

import static frc.robot.Util.logf;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimeLightPoseSubsystem;

public class ResetOdometryWithCameraCommand extends CommandBase {

    LimeLightPoseSubsystem limeLightPoseSubsystem;
    double areaThreshold;
    int iteration = 0;
    int iterationsToWait;
    boolean completed = false;
    NetworkTableEntry ta;

    public ResetOdometryWithCameraCommand(LimeLightPoseSubsystem lightPoseSubsystem,
                                          double areaThreshold,
                                          int iterationsToWait) {
        this.limeLightPoseSubsystem = lightPoseSubsystem;
        this.areaThreshold = areaThreshold;
        this.iterationsToWait = iterationsToWait;
    }

    @Override
    public void initialize() {
        ta = NetworkTableInstance.getDefault().getTable(limeLightPoseSubsystem.cameraId).getEntry("ta");
        // limeLightPoseSubsystem.setCurrentPose(new Pose2d(1.89, 0.5, new Rotation2d(Math.toRadians(180))));
    }

    @Override
    public void execute() {
        String pipeLine = "botpose_wpiblue";//(Robot.alliance == Alliance.Red) ? "botpose_wpired" : "botpose_wpiblue";
        double llPose[] = NetworkTableInstance.getDefault().getTable(limeLightPoseSubsystem.cameraId).getEntry(pipeLine)
                .getDoubleArray(new double[6]);            
        double area = ta.getDouble(0);
        if (area > areaThreshold) {
            double cameraAngle = Math.toRadians(llPose[5]);
            Pose2d visionPose = new Pose2d(llPose[0], llPose[1], new Rotation2d(cameraAngle));
            limeLightPoseSubsystem.setCurrentPose(visionPose);
            logf("Odometry reset with vision successful %s\n", visionPose.toString());
            completed = true;
        } 
        iteration++;
    }

    @Override
    public boolean isFinished() {
        return completed || (iteration > iterationsToWait);
    }
}
