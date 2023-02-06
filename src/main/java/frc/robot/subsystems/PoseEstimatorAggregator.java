package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public class PoseEstimatorAggregator implements Supplier<Pose2d> {
    public PoseEstimatorSubsystem poseEstimators[];
    private final Field2d field2d = new Field2d();

    public PoseEstimatorAggregator(PoseEstimatorSubsystem poseEstimators[]) {
        this.poseEstimators = poseEstimators;
        ShuffleboardTab tab = Shuffleboard.getTab("Vision Agreegator");
        tab.addString("Pose", this::getFomattedPose).withPosition(0, 0).withSize(2, 0);
        tab.add("Field", field2d).withPosition(2, 0).withSize(6, 4);
    }

    private String getFomattedPose() {
        var pose = get();
        return String.format("(%.2f, %.2f) %.2f degrees",
                pose.getX(),
                pose.getY(),
                pose.getRotation().getDegrees());
    }

    @Override
    public Pose2d get() {
        double x = 0;
        double y = 0;
        double angle = 0;
        double subsystemWeight[] = new double[poseEstimators.length];
        double total = 0;
        for (int i = 0; i < poseEstimators.length; ++i) {
            subsystemWeight[i] = poseEstimators[i].getAvg();
            total += subsystemWeight[i];
        }
        for (int i = 0; i < poseEstimators.length; ++i) {
            Pose2d pose = poseEstimators[i].getCurrentPose();
            x += pose.getX() * subsystemWeight[i];
            y += pose.getY() * subsystemWeight[i];
            angle += pose.getRotation().getRadians() * subsystemWeight[i];
        }
        Pose2d pose = new Pose2d(x / total, y / total, new Rotation2d(angle / total));
        drawField(pose);
        return pose;
    }

    public void drawField(Pose2d pose) {
        field2d.setRobotPose(pose);
        // Conversion so robot appears where it actually is on field instead of always
        // on blue.
        if (DriverStation.getAlliance() == Alliance.Red) {
            field2d.setRobotPose(new Pose2d(FieldConstants.fieldLength - pose.getX(),
                    FieldConstants.fieldWidth - pose.getY(),
                    new Rotation2d(pose.getRotation().getRadians() + Math.PI)));
        } else {
            field2d.setRobotPose(pose);
        }
    }
}
