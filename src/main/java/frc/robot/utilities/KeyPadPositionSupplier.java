package frc.robot.utilities;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * This class is used to provide the robot the position on the field
 * to which it needs to go to when a button in the key pad is pressed.
 * It uses the state to know which set of targets it will go after.
 */
public class KeyPadPositionSupplier implements Supplier<Pose2d> {
    /**
     * The state can be 0, 1, or 2.
     * 0 is the left most group of 9 nodes. 1 is the center, and 2 is the right most.
     */
    public static int state = 0;
    int pos;
    public final static Pose2d driveTrainPoseTargets[] = new Pose2d[] {
        new Pose2d(new Translation2d(1.88, 3.09), new Rotation2d(Math.toRadians(180))), // this worked!       
        new Pose2d(new Translation2d(1.88, 3.59), new Rotation2d(Math.toRadians(180))),
        new Pose2d(new Translation2d(1.88, 4.09), new Rotation2d(Math.toRadians(180))),

        new Pose2d(new Translation2d(1.88, 2.88), new Rotation2d(Math.toRadians(180))), // this worked!       
        new Pose2d(new Translation2d(1.88, 0.0), new Rotation2d(0,0)),
        new Pose2d(new Translation2d(1.88, 0.0), new Rotation2d(0,0)),

        new Pose2d(new Translation2d(1.88, 2.88), new Rotation2d(Math.toRadians(180))), // this worked!       
        new Pose2d(new Translation2d(1.88, 0.0), new Rotation2d(0,0)),
        new Pose2d(new Translation2d(1.88, 0.0), new Rotation2d(0,0))
    };

    public KeyPadPositionSupplier(int pos) {
        this.pos = pos;
    }

    @Override
    public Pose2d get() {
        return driveTrainPoseTargets[state * 3 + pos % 3];
    }

}
