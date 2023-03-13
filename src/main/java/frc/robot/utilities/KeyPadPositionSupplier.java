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
        // tag id 3, 6 (tested 6 and it worked)
        new Pose2d(new Translation2d(1.72, 3.046), new Rotation2d(Math.toRadians(180))),
        new Pose2d(new Translation2d(1.72, 3.586), new Rotation2d(Math.toRadians(180))),
        new Pose2d(new Translation2d(1.72, 4.126), new Rotation2d(Math.toRadians(180))),

        // tag id 2, 7
        new Pose2d(new Translation2d(1.72, 4.58), new Rotation2d(Math.toRadians(180))),
        new Pose2d(new Translation2d(1.72, 5.24), new Rotation2d(Math.toRadians(180))),        
        new Pose2d(new Translation2d(1.72, 5.76), new Rotation2d(Math.toRadians(180))), 
        
         // tag id 1, 8
         new Pose2d(new Translation2d(1.72, 6.398), new Rotation2d(Math.toRadians(180))),        
         new Pose2d(new Translation2d(1.72, 6.938), new Rotation2d(Math.toRadians(180))),
         new Pose2d(new Translation2d(1.72, 7.478), new Rotation2d(Math.toRadians(180)))
        
    };

    public KeyPadPositionSupplier(int pos) {
        this.pos = pos;
    }


    @Override
    public Pose2d get() {
        var alliance = DriverStation.getAlliance();
        Pose2d pose;
        if (alliance == Alliance.Red) {
            pose = driveTrainPoseTargets[(2-state) * 3 + 2-(pos % 3)];
        } else {
            pose = driveTrainPoseTargets[state * 3 + pos % 3];
            pose = new Pose2d(pose.getX(), FIELD_WIDTH - pose.getY(), pose.getRotation());
        }
        return pose;
    }

}
