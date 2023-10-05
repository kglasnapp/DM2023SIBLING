package frc.robot.subsystems;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import static frc.robot.Util.logf;

import java.util.function.Supplier;

import edu.wpi.first.math.Vector;

public class LimeLightPoseSubsystem extends SubsystemBase implements Supplier<Pose2d> {
    private final SwerveDrivePoseEstimator poseEstimator;
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    NetworkTableEntry tv = table.getEntry("tv");
    //NetworkTableEntry timestamp = table.getEntry("timestamp");
    ShuffleboardTab tab;
    private final Field2d field2d = new Field2d();
    Pose2d pose = new Pose2d();
    private static final Vector<N3> stateStdDevs = VecBuilder.fill(0.2, 0.2, Units.degreesToRadians(5));
    private static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(10));
    DrivetrainSubsystem drivetrainSubsystem;

    public LimeLightPoseSubsystem(DrivetrainSubsystem drivetrainSubsystem) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        tab = Shuffleboard.getTab("Vision LimeLight");
        tab.addString("Pose", this::getFomattedPose).withPosition(0, 0).withSize(2, 0);
        tab.add("Field", field2d).withPosition(2, 0).withSize(6, 4);
        poseEstimator = new SwerveDrivePoseEstimator(
                DrivetrainSubsystem.m_kinematics,
                drivetrainSubsystem.getGyroscopeRotation(),
                drivetrainSubsystem.getModulePositions(),
                new Pose2d(),
                stateStdDevs,
                visionMeasurementStdDevs);
    }

    double camearaToYawAdjustment = 0;
    Rotation2d yaw = new Rotation2d();

    @Override
    public void periodic() {
        //read values periodically
        yaw = drivetrainSubsystem.getGyroscopeRotation();
        if (tv.getDouble(0.0) == 1.0) {
            // x and y are in degrees
            double x = tx.getDouble(0.0);
            double y = ty.getDouble(0.0);
            double area = ta.getDouble(0.0);
            if (Robot.count % 10 == 0) {
                //post to smart dashboard periodically
                SmartDashboard.putNumber("LimeLX", x);
                SmartDashboard.putNumber("LimeLY", y);
                SmartDashboard.putNumber("LimeLArea", area);
            }

            String pipeLine = (Robot.alliance == Alliance.Red) ? "botpose_wpired" : "botpose_wpiblue";
            double llPose[] = NetworkTableInstance.getDefault().getTable("limelight").getEntry(pipeLine)
                    .getDoubleArray(new double[6]);
            double cameraAngle = Math.toRadians(llPose[5]);
            camearaToYawAdjustment = cameraAngle - yaw.getRadians();
            Pose2d visionPose = new Pose2d(llPose[0], llPose[1], new Rotation2d(cameraAngle));
            double timeS = RobotController.getFPGATime() / 1000000.0;
            poseEstimator.addVisionMeasurement(visionPose, timeS);
        }
        yaw = new Rotation2d(yaw.getRadians() - camearaToYawAdjustment);
        poseEstimator.update(yaw,
                drivetrainSubsystem.getModulePositions());
        pose = poseEstimator.getEstimatedPosition();
        if (Robot.count % 10 == 0) {
            field2d.setRobotPose(pose);
        }
    }

    public Pose2d getPose() {
        return pose;
    }

    private String getFomattedPose() {
        String s = String.format("(%.2f, %.2f) %.2f degrees",
                pose.getX(),
                pose.getY(),
                pose.getRotation().getDegrees());
        if (Robot.count % 250 == 0) {
            logf("LL Pose %s yaw:%.2f\n", s, yaw.getDegrees());
        }
        return s;
    }

    @Override
    public Pose2d get() {
        return pose;
    }
}