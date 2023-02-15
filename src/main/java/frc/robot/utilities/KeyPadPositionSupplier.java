package frc.robot.utilities;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * This class is used to provide the robot the position on the field
 * to which it needs to go to when a button in the key pad is pressed.
 * It uses the state to know which set of targets it will go after.
 */
public class KeyPadPositionSupplier implements Supplier<Pose2d> {
    public static final double FIELD_WIDTH = 8.0137;
    /**
     * The state can be 0, 1, or 2.
     * 0 is the left most group of 9 nodes. 1 is the center, and 2 is the right most.
     */
    public static int state = 0;
    int pos;
    final static double changeX = -0.04;
    final static double changeY = -0.12;
    public final static Pose2d driveTrainPoseTargets[] = new Pose2d[] {
        new Pose2d(new Translation2d(1.88 + changeX, 3.09 + changeY), new Rotation2d(Math.toRadians(180))),        
        new Pose2d(new Translation2d(2.05 + changeX, 3.59 + changeY), new Rotation2d(Math.toRadians(180))),
        new Pose2d(new Translation2d(2.00 + changeX, 4.09 ), new Rotation2d(Math.toRadians(180))),

        new Pose2d(new Translation2d(1.83, 3.24), new Rotation2d(Math.toRadians(180))), 
        new Pose2d(new Translation2d(1.83, 2.67), new Rotation2d(Math.toRadians(180))),
        new Pose2d(new Translation2d(1.83, 2.10), new Rotation2d(Math.toRadians(180))),

        new Pose2d(new Translation2d(1.84, -0.27 + changeY), new Rotation2d(Math.toRadians(180))), 
        new Pose2d(new Translation2d(2.01, 0.23 + changeY), new Rotation2d(0,0)),
        new Pose2d(new Translation2d(2.00 + changeX, 0.73), new Rotation2d(0,0))
    };

    

    public KeyPadPositionSupplier(int pos) {
        this.pos = pos;
    }


    @Override
    public Pose2d get() {
        var alliance = DriverStation.getAlliance();
        Pose2d pose;
        if (alliance == Alliance.Blue) {
            pose = driveTrainPoseTargets[state * 3 + pos % 3];
        } else {
            pose = driveTrainPoseTargets[state * 3 + pos % 3];
            pose = new Pose2d(pose.getX(), FIELD_WIDTH - pose.getY(), pose.getRotation());
        }
        return pose;
    }

}
