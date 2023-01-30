package frc.robot.subsystems;

import static edu.wpi.first.math.util.Units.degreesToRadians;

import java.util.HashMap;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.utilities.Util.logf;
import frc.robot.Robot;
import frc.robot.utilities.RunningAverage;

public class PoseEstimatorSubsystem extends SubsystemBase {

  private final PhotonCamera photonCameras[];
  private final DrivetrainSubsystem drivetrainSubsystem;
  private RunningAverage averageTime = new RunningAverage(10);

  private static HashMap<String, Pose3d> targets = new HashMap<>();

  static {
    // targets.put("1", new Pose3d(2.37,2.51, 0.3, new Rotation3d(0, 0,
    // degreesToRadians(180.0))));
    // targets.put("2", new Pose3d(-0.055, 2.51, 0.3, new Rotation3d(0, 0,
    // degreesToRadians(0.0))));
    targets.put("3", new Pose3d(15.51, 4.42, 0.46, new Rotation3d(0, 0, degreesToRadians(180.0))));
    // targets.put("4", new Pose3d(2.37, 0.53, 0.3, new Rotation3d(0, 0,
    // degreesToRadians(180.0))));
    // targets.put("5", new Pose3d(1.13, -0.07, 0.3, new Rotation3d(0, 0,
    // degreesToRadians(90.0))));
    // targets.put("6", new Pose3d(1.13, 2.96, 0.3, new Rotation3d(0, 0,
    // degreesToRadians(-90.0))));
  }

  // Kalman Filter Configuration. These can be "tuned-to-taste" based on how much
  // you trust your various sensors. Smaller numbers will cause the filter to
  // "trust" the estimate from that particular component more than the others.
  // This in turn means the particualr component will have a stronger influence
  // on the final pose estimate.

  /**
   * Standard deviations of model states. Increase these numbers to trust your
   * model's state estimates less. This
   * matrix is in the form [x, y, theta]ᵀ, with units in meters and radians, then
   * meters.
   */
  private static final Vector<N3> stateStdDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));

  /**
   * Standard deviations of the vision measurements. Increase these numbers to
   * trust global measurements from vision
   * less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and
   * radians.
   */
  private static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(10));

  private final SwerveDrivePoseEstimator poseEstimator;

  private final Field2d field2d = new Field2d();
  // we create labels for the smartdashboard
  // avoiding garbage collection
  private final String TARGET_LABEL[] = {
      "tar1", "tar2"
  };
  private final String AMBIGUITY_LABEL[] = {
      "AMB1", "AMB2"
  };
  private final String TAG_LABEL[] = {
      "tag1", "tag2"
  };
  private final String CAM_TO_TARGET_X_LABEL[] = {
      "CAM to tar X1", "CAM to tar X2"
  };
  private final String CAM_TO_TARGET_Y_LABEL[] = {
      "CAM to tar Y1", "CAM to tar Y2"
  };
  private final String CAM_X_LABEL[] = {
      "CAM X1", "CAM X2"
  };
  private final String CAM_Y_LABEL[] = {
      "CAM Y1", "CAM Y2"
  };
  private final String V_X_LABEL[] = {
      "V X1", "V X2"
  };
  private final String V_Y_LABEL[] = {
      "V Y1", "V Y2"
  };

  private double previousPipelineTimestamp[];

  private final static boolean enableReportPoseEstimator = false;
  private final static boolean enableReportTarget = true;
  private final static boolean enableReportVisionPose = true;
  private final Transform3d cameraPositions[];

  public PoseEstimatorSubsystem(PhotonCamera photonCameras[], Transform3d cameraPositions[], DrivetrainSubsystem drivetrainSubsystem) {
    this.photonCameras = photonCameras;
    this.cameraPositions = cameraPositions;
    previousPipelineTimestamp = new double[photonCameras.length];
    this.drivetrainSubsystem = drivetrainSubsystem;

    ShuffleboardTab tab = Shuffleboard.getTab("Vision");

    poseEstimator = new SwerveDrivePoseEstimator(
        DrivetrainSubsystem.m_kinematics,
        drivetrainSubsystem.getGyroscopeRotation(),
        drivetrainSubsystem.getModulePositions(),
        new Pose2d(),
        stateStdDevs,
        visionMeasurementStdDevs);

    tab.addString("Pose", this::getFomattedPose).withPosition(0, 0).withSize(2, 0);
    tab.add("Field", field2d).withPosition(2, 0).withSize(6, 4);
  }

  final static double AMBIGUITY = 0.2;

  public void processCamera(int cameraId, PhotonCamera photonCamera) {
    PhotonPipelineResult pipelineResult = photonCamera.getLatestResult();
    double resultTimestamp = pipelineResult.getTimestampSeconds();
    reportTarget(cameraId, pipelineResult); // update smartdashboard
    if (resultTimestamp != previousPipelineTimestamp[cameraId] && pipelineResult.hasTargets()) {
      previousPipelineTimestamp[cameraId] = resultTimestamp;
      PhotonTrackedTarget target = pipelineResult.getBestTarget();
      if (target.getPoseAmbiguity() <= AMBIGUITY) {
        int fiducialId = target.getFiducialId();
        Pose3d targetPose = targets.get("" + fiducialId);
        if (targetPose != null) {
          Transform3d camToTarget = target.getBestCameraToTarget();
          Pose3d camPose = targetPose.transformBy(camToTarget.inverse());
          Pose3d visionMeasurement = camPose.transformBy(cameraPositions[cameraId]);
          poseEstimator.addVisionMeasurement(visionMeasurement.toPose2d(), resultTimestamp);
          reportVisionPose(cameraId, fiducialId, camToTarget, camPose, visionMeasurement);
        }
      }
    }
  }

  /**
   * This method just pushes info into the Smart Dashboard about
   * the the target being visible or not.
   * 
   * @param cameraId
   * @param pipelineResult
   */
  public void reportTarget(int cameraId, PhotonPipelineResult pipelineResult) {
    if (Robot.count % 10 == 3 && enableReportTarget) {
      SmartDashboard.putBoolean(TARGET_LABEL[cameraId], pipelineResult.hasTargets() &&
          pipelineResult.getBestTarget().getPoseAmbiguity() < AMBIGUITY);
      if (pipelineResult.hasTargets()) {
        SmartDashboard.putNumber(AMBIGUITY_LABEL[cameraId], pipelineResult.getBestTarget().getPoseAmbiguity());
      } else {
        SmartDashboard.putNumber(AMBIGUITY_LABEL[cameraId], 9999);
      }
    }
  }

  /**
   * This method pushes to the Smart Dashboard different parameters to help
   * debug the camera readings
   * 
   * @param cameraId
   * @param fiducialId
   * @param camToTarget
   * @param camPose
   * @param visionMeasurement
   */
  public void reportVisionPose(int cameraId, int fiducialId, Transform3d camToTarget, Pose3d camPose,
      Pose3d visionMeasurement) {
    if (Robot.count % 15 == 0 && enableReportVisionPose ) {
      SmartDashboard.putNumber(TAG_LABEL[cameraId], fiducialId);
      SmartDashboard.putNumber(CAM_TO_TARGET_X_LABEL[cameraId], camToTarget.getX());
      SmartDashboard.putNumber(CAM_TO_TARGET_Y_LABEL[cameraId], camToTarget.getY());
      SmartDashboard.putNumber(CAM_X_LABEL[cameraId], camPose.getX());
      SmartDashboard.putNumber(CAM_Y_LABEL[cameraId], camPose.getY());
      SmartDashboard.putNumber(V_X_LABEL[cameraId], visionMeasurement.toPose2d().getX());
      SmartDashboard.putNumber(V_Y_LABEL[cameraId], visionMeasurement.toPose2d().getY());

    //   logf(
    //       "VISION ONLY: Pose Est ID:%d camtotarg<%.2f,%.2f,%.2f> camPose<%.2f,%.2f,%.2f> vision<%.2f,%.2f,%.2f> RunTime:%.2f\n",
    //       fiducialId, camToTarget.getX(), camToTarget.getY(),
    //       Math.toDegrees(camToTarget.getRotation().getAngle()),
    //       camPose.getX(), camPose.getY(), Math.toDegrees(camPose.getRotation().getAngle()),
    //       visionMeasurement.getX(), visionMeasurement.getY(),
    //       Math.toDegrees(visionMeasurement.getRotation().getAngle()),
    //       averageTime.getAverage());
     }
  }

  /**
   * This method pushes to the Smart Dashboard about the pose estimator
   */
  public void reportPoseEstimator() {
    // if (Robot.count % 15 == 9 && enableReportPoseEstimator) {
    if (Robot.count % 15 == 0 ) {
      SmartDashboard.putNumber("Est X Pos", poseEstimator.getEstimatedPosition().getX());
      SmartDashboard.putNumber("Est Y Pos", poseEstimator.getEstimatedPosition().getY());
      SmartDashboard.putNumber("Est Angle", poseEstimator.getEstimatedPosition().getRotation().getDegrees());
      // logf("VISION+ODOMETRY est x = %.2f y = %.2f angle=%.2f\n",
      // poseEstimator.getEstimatedPosition().getX(),
      // poseEstimator.getEstimatedPosition().getY(),
      // poseEstimator.getEstimatedPosition().getRotation().getDegrees());
    }
  }
 

  @Override
  public void periodic() {
    long startTime = RobotController.getFPGATime();
    // Update pose estimator with the best visible target
    for (int cameraId = 0; cameraId < photonCameras.length; ++cameraId) {
      processCamera(cameraId, photonCameras[cameraId]);
    }

    // Update pose estimator with drivetrain sensors
    poseEstimator.update(
        drivetrainSubsystem.getGyroscopeRotation(),
        drivetrainSubsystem.getModulePositions());

    field2d.setRobotPose(getCurrentPose());

    reportPoseEstimator();

    long totalTime = RobotController.getFPGATime() - startTime;
    averageTime.add(totalTime / 1000);
  }

  private String getFomattedPose() {
    var pose = getCurrentPose();
    return String.format("(%.2f, %.2f) %.2f degrees",
        pose.getX(),
        pose.getY(),
        pose.getRotation().getDegrees());
  }

  public Pose2d getCurrentPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * Resets the current pose to the specified pose. This should ONLY be called
   * when the robot's position on the field is known, like at the beginning of
   * a match.
   * 
   * @param newPose new pose
   */
  public void setCurrentPose(Pose2d newPose) {
    poseEstimator.resetPosition(
        drivetrainSubsystem.getGyroscopeRotation(),
        drivetrainSubsystem.getModulePositions(),
        newPose);
  }

  /**
   * Resets the position on the field to 0,0 0-degrees, with forward being
   * downfield. This resets
   * what "forward" is for field oriented driving.
   */
  public void resetFieldPosition() {
    setCurrentPose(new Pose2d());
  }

}
