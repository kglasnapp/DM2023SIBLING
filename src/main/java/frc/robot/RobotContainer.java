// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Util.logf;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.BalanceCommand;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.DefaultElevatorCommand;
import frc.robot.commands.DefaultGrabberCommand;
import frc.robot.commands.PositionCommand;
import frc.robot.commands.RotateCommand;
import frc.robot.commands.SetModeConeCube;
import frc.robot.commands.StraightPathCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GrabberTiltSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.LimeLightPoseSubsystem;
import frc.robot.subsystems.PoseEstimatorAggregator;
import frc.robot.utilities.SwerveModule;
//import frc.robot.Autonomous;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // robotIDCheck.get returns true for the sibling, false for
  final DigitalInput robotIDCheck = new DigitalInput(0);

  public final GrabberTiltSubsystem grabberSubsystem = new GrabberTiltSubsystem(this);
  //private final LimeLightPose limeLightPose = new LimeLightPose();
  public final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  public final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem(grabberSubsystem);
  public final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();
  public final static LedSubsystem leds = new LedSubsystem();
  private final static CommandXboxController driveController = new CommandXboxController(2);
  private final static CommandXboxController operatorController = new CommandXboxController(3);
  
  public boolean USBCamera = false;
  public final BalanceCommand balanceCommand = new BalanceCommand(drivetrainSubsystem);
  private SlewRateLimiter sLX = new SlewRateLimiter(9);
  private SlewRateLimiter sLY = new SlewRateLimiter(9);
  private SlewRateLimiter sRX = new SlewRateLimiter(1);
  public static SendableChooser<Command> autonomousChooser = new SendableChooser<>();
  public static boolean smartForElevator = true;
  public static RobotMode robotMode;
  public static boolean smartDashBoardForElevator = true;
  public LimeLightPoseSubsystem limeLightPoseSubsystemLeft;
  public LimeLightPoseSubsystem limeLightPoseSubsystemRight;

  public static RobotContainer instance;

  public static ShowPID showPID = ShowPID.TILT;
  
  public Autonomous autotonomous;

  public static enum RobotMode {
    Cone, Cube
  }

  public static enum ShowPID {
    NONE, ELEVATOR, TILT
  }

  public static enum OperatorButtons {
    HOME(3), CUBE(2), CONE(1), GROUND(4), CHUTE(5), SHELF(6), LOW(7), MIDDLE(8), HIGH(9), ELECTRIALHOME(15);

    public final int value;

    private OperatorButtons(int value) {
      this.value = value;
    }
  }

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    instance = this;
    // Set the default Robot Mode to Cube
    setMode(RobotMode.Cube);
    grabberSubsystem.setDefaultCommand(new DefaultGrabberCommand(grabberSubsystem, intakeSubsystem, driveController));
    Command defaulElevatorCommand = new DefaultElevatorCommand(elevatorSubsystem, grabberSubsystem, driveController);
    elevatorSubsystem.setDefaultCommand(defaulElevatorCommand);

    drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
        drivetrainSubsystem,
        () -> (SwerveModule.getPowerRatio() == 1 ? -modifyAxis((sLY.calculate(driveController.getLeftY())))
            * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND
            : -modifyAxis((driveController.getLeftY()))
                * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND),
        () -> (SwerveModule.getPowerRatio() == 1 ? -modifyAxis((sLX.calculate(driveController.getLeftX())))
            * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND
            : -modifyAxis((driveController.getLeftX()))
                * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND),
        () -> (SwerveModule.getPowerRatio() == 1
            ? -modifyAxis((sRX.calculate(driveController.getRightX())))
                * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND
            : -modifyAxis((driveController.getRightX()))
                * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND),
        driveController.y()));// Set precision based upon left bumper
    limeLightPoseSubsystemLeft = new LimeLightPoseSubsystem(drivetrainSubsystem, "limelight-left");
    limeLightPoseSubsystemRight = new LimeLightPoseSubsystem(drivetrainSubsystem, "limelight-right");
    configureButtonBindings();
    configureDashboard();
    autotonomous = new Autonomous(this, drivetrainSubsystem, intakeSubsystem);
  }

  public double squareWithSign(double v) {
    return (v < 0) ? -(v * v) : (v * v);
  }

  private void configureDashboard() {
  }

  public static double getRightTrigger() {
    return driveController.getRightTriggerAxis();
  }

  public static double getLeftTrigger() {
    return driveController.getLeftTriggerAxis();
  }

  public static boolean getRightBumper() {
    return driveController.getHID().getRawButton(6);
  }

  public static boolean getLeftBumper() {
    return driveController.getHID().getRawButton(5);
  }

  public static int getDriverPov() {
    return driveController.getHID().getPOV();
  }

  public static boolean getBack() {
    return driveController.getHID().getBackButton();
  }

  public void setMode(RobotMode mode) {
    robotMode = mode;
    logf("Set Robot Mode: %s\n", mode);
    SmartDashboard.putString("Mode", robotMode.toString());
    leds.setRobotModeLeds();
  }

  public Pose2d getLLPose() {
    return limeLightPoseSubsystemLeft.getPose();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // new Button(m_controller::getAButton).whileHeld(holonomicTargetCommand);
    // new Button(m_controller::getBButton).whileHeld(chaseTagCommand);
    // Back button zeros the gyroscope);
    driveController.back().whileTrue(new RunCommand(new Runnable() {
      public void run() {
        drivetrainSubsystem.zeroGyroscope();
      }
    }));

    //driveController.back().onTrue(new PositionCommand(this, OperatorButtons.LOW));

    driveController.start().whileTrue(new RunCommand(new Runnable() {
      public void run() {
        balanceCommand.zeroGyroscope();
      }
    }));

    driveController.start().whileTrue(Commands.runOnce(new Runnable() {
      public void run() {
        if (showPID == ShowPID.TILT) {
          showPID = ShowPID.ELEVATOR;
        } else {
          showPID = ShowPID.TILT;
        }
        logf("Change PID control to %s\n", showPID);
      }
    }));
    // y Button will rotate the Robot 180 degrees
    driveController.y().onTrue(new RotateCommand(drivetrainSubsystem));
    // driveController.a().whileTrue(
    //    new TrajectoryCommand("/home/lvuser/deploy/Red 3.wpilib.json", drivetrainSubsystem, limeLightPoseSubsystem));
    // Red 3
    
    driveController.b().whileTrue(
      new CommandBase() {
        @Override
        public void initialize() {

            String pipeLine = "botpose_wpiblue";//(Robot.alliance == Alliance.Red) ? "botpose_wpired" : "botpose_wpiblue";
            double llPose[] = NetworkTableInstance.getDefault().getTable(limeLightPoseSubsystemLeft.cameraId).getEntry(pipeLine)
                    .getDoubleArray(new double[6]);
            double cameraAngle = Math.toRadians(llPose[5]);
            Pose2d visionPose = new Pose2d(llPose[0], llPose[1], new Rotation2d(cameraAngle));
            limeLightPoseSubsystemLeft.setCurrentPose(visionPose);
            // limeLightPoseSubsystem.setCurrentPose(new Pose2d(1.89, 0.5, new Rotation2d(Math.toRadians(180))));
        }

        @Override
        public boolean isFinished() {
            return true;
        }
    }.andThen(


      new StraightPathCommand(drivetrainSubsystem, limeLightPoseSubsystemLeft, new Pose2d(6.50,4.57,new Rotation2d(Math.toRadians(180)))))); 
    // Autonomous.get2PiecesCommand(this,
    //                     this.limeLightPoseSubsystemRight,
    //                     "/home/lvuser/deploy/Red1.wpilib.json",
    //                     "/home/lvuser/deploy/Red1Return.wpilib.json",
    //                     // TODO need to update final position
    //                     new Pose2d(10.2, 0.9, new Rotation2d(Math.toRadians(180)))));
    
    
    // Autonomous.get2PiecesCommand(this, limeLightPoseSubsystemRight,"/home/lvuser/deploy/Blue8.wpilib.json",        
      // "/home/lvuser/deploy/Blue8Return.wpilib.json", new Pose2d(6.83, 0.9,new Rotation2d(Math.toRadians(0)))));
    // driveController.b().whileTrue(Autonomous.get2PiecesCommand(this, "/home/lvuser/deploy/Blue8.wpilib.json",
    //     "/home/lvuser/deploy/Blue8Return.wpilib.json", new Pose2d(6.83, 0.9, new Rotation2d(Math.toRadians(0)))));
    operatorController.button(OperatorButtons.LOW.value).onTrue(new PositionCommand(this, OperatorButtons.LOW));
    operatorController.button(OperatorButtons.MIDDLE.value).onTrue(new PositionCommand(this, OperatorButtons.MIDDLE));
    operatorController.button(OperatorButtons.HIGH.value).onTrue(new PositionCommand(this, OperatorButtons.HIGH));
    operatorController.button(OperatorButtons.HOME.value).onTrue(new PositionCommand(this, OperatorButtons.HOME));

    operatorController.button(OperatorButtons.GROUND.value).onTrue(new PositionCommand(this, OperatorButtons.GROUND));
    operatorController.button(OperatorButtons.CHUTE.value).onTrue(new PositionCommand(this, OperatorButtons.CHUTE));
    operatorController.button(OperatorButtons.SHELF.value).onTrue(new PositionCommand(this, OperatorButtons.SHELF));

    operatorController.button(OperatorButtons.CONE.value).onTrue(new SetModeConeCube(RobotMode.Cone));
    operatorController.button(OperatorButtons.CUBE.value).onTrue(new SetModeConeCube(RobotMode.Cube));

  }

  // Command a = RobotContainer.runOnce(()->{ setMode(RobotMode.Cone);});

  /** Grabs the hatch. */
  // public CommandBase grabHatchCommand() {
  // implicitly require `this`
  // return this.runOnce(() -> m_hatchSolenoid.set(kForward));
  //   }

  public Pose2d getPickupPose(boolean right, int phase) {
    var alliance = DriverStation.getAlliance();
    Pose2d pose = null;
    if (alliance == Alliance.Blue) {
      if (right) {
        pose = new Pose2d(new Translation2d(15.14, 5.95), new Rotation2d(Math.toRadians(0)));
      } else {
        pose = new Pose2d(new Translation2d(15.14, 7.50), new Rotation2d(Math.toRadians(0)));
      }
      if (phase == 1) {
        pose = new Pose2d(pose.getX() - 0.36, pose.getY(), pose.getRotation());
      }
    } else {
      if (right) {
        pose = new Pose2d(new Translation2d(15.14, 0.55), new Rotation2d(Math.toRadians(0)));
      } else {
        pose = new Pose2d(new Translation2d(15.14, 1.88), new Rotation2d(Math.toRadians(0)));
      }
      if (phase == 1) {
        pose = new Pose2d(pose.getX() - 0.36, pose.getY(), pose.getRotation());
      }
    }
    return pose;
  }

  Supplier<Pose2d> getPoseEstimatorForTarget(PoseEstimatorAggregator poseEstimatorAggregator, int pos) {
    return poseEstimatorAggregator;
  }

  Pose2d getFinalPoseForCase1(int state) {
    if (DriverStation.getAlliance() == Alliance.Blue) {
      if (state == 0) {
        return new Pose2d(6.4,
            4.5,
            new Rotation2d(Math.toRadians(180)));
      } else {
        return new Pose2d(6.4,
            0.92,
            new Rotation2d(Math.toRadians(180)));
      }
    } else {
      if (state == 0) {
        return new Pose2d(6.4,
            1.096, // just changed this value at 4:54
            new Rotation2d(Math.toRadians(180)));
      } else {
        return new Pose2d(6.4,
            3.42,
            new Rotation2d(Math.toRadians(180)));
      }
    }
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxis(double value) {
    value = deadband(value, 0.08);
    value = Math.copySign(value * value, value); // Square the axis
    return value;
  }
}
